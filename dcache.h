/*
 * dcache.h
 *
 * data cache
 * (c) JCGV, junio del 2022
 *
 */
#ifdef BOARD_ESP32
#define RAM_KB	    128
#else
#define RAM_KB	    32
#endif

// backing RAM
#define SRAM_PAGE   32
static uint8_t	    ram[RAM_KB * 1024];

static void ram_readPage (uint32_t addr, uint8_t *p) {
    for (unsigned i = 0; i < SRAM_PAGE; i++)
	*p++ = ram[addr + i];
}

static void ram_writePage (uint32_t addr, const uint8_t *p) {
    for (unsigned i = 0; i < SRAM_PAGE; i++)
	ram[addr + i] = *p++;
}

static uint8_t * dma_at (uint32_t addr) {
    return &ram[addr];
}

static void dma_set (uint32_t addr, byte b) {
    ram[addr] = b;
}

static void dma_set4 (uint32_t addr, word32 w) {
    dma_set (addr,     w >> 24);
    dma_set (addr + 1, w >> 16);
    dma_set (addr + 2, w >>  8);
    dma_set (addr + 3, w >>  0);
}

static word32 dma_get4 (uint32_t addr) {
    word32 b0 = ram[addr + 0] << 24;
    word32 b1 = ram[addr + 1] << 16;
    word32 b2 = ram[addr + 2] << 8;
    word32 b3 = ram[addr + 3];
    return b0 | b1 | b2 | b3;
}

#define WAYS	    2	    // 2/4 WAYS
#define DSIZE	    4096
#define NLINES	    (DSIZE / SRAM_PAGE / WAYS)

#define FLAG_FREE   0x01
#define FLAG_DIRTY  0x02

static byte dcache_raw[DSIZE];

static struct {
    word32  tag[WAYS];
    byte    lru;
} dlines[NLINES];

static word32 dcache_use, dcache_hit, dcache_load, dcache_evicted, dcache_wback;

static void dcache_init (void) {
    dcache_use = dcache_hit = dcache_load = dcache_evicted = dcache_wback;
    for (unsigned i = 0; i < NLINES; i++)
	for (unsigned w = 0; w < WAYS; w++)
	    dlines[i].tag[w] = FLAG_FREE;
}

void line_dump (unsigned line, const char *title) {
    sprintf (sbuf, "\t|line %02x LRU %02x %s:", line, dlines[line].lru, title);
    Serial.println (sbuf);
    for (unsigned w = 0; w < WAYS; w++) {
	word32 t = dlines[line].tag[w];
	int    l = sprintf (sbuf, "\t| #%u ", w);
	if (t & FLAG_FREE)
	    l += sprintf (sbuf + l, "FREE");
	else {
	    l += sprintf (sbuf + l, "%c%08lx", t & FLAG_DIRTY ? '!' : ' ', t);
	    byte *data = &dcache_raw[line * WAYS * SRAM_PAGE + w * SRAM_PAGE];
	    for (int i = 0; i < SRAM_PAGE; i++)
		l += sprintf (sbuf + l, "%s%02x", i % 4 == 0 ? " " : "", data[i]);
	}
	Serial.println (sbuf);
    }
}

static void dcache_inv (word32 addr, word32 bytes) {
    word32 top	= addr + bytes;
    word32 mask = SRAM_PAGE - 1;
    //sprintf (sbuf, "\t| inv %08lx to %08lx", addr, top); Serial.println (sbuf);
    for (int i = 0; i < NLINES; i++)
	for (int w = 0; w < WAYS; w++) {
	    word32 t = dlines[i].tag[w];
	    if (! (t & FLAG_FREE)) {
		word32 at = t & ~mask;
		if (at >= addr && (at + SRAM_PAGE) <= top) {
		    // write back
		    if (t & FLAG_DIRTY) {
			dcache_wback++;
			byte *p = &dcache_raw[i * WAYS * SRAM_PAGE + w * SRAM_PAGE];
			ram_writePage (at, p);
		    }

		    // set free
		    //line_dump (i, "inv");
		    dlines[i].tag[w] |= FLAG_FREE;
		}
	    }
	}
}

static word32 dcache (word32 addr, bool write, unsigned bytes, word32 value) {
    // get line
    word32   mask = SRAM_PAGE - 1;
    word32   tag  = addr & ~mask;
    unsigned line = (addr / SRAM_PAGE) % NLINES;
    //sprintf (sbuf, "\t| addr %08lx line %u/%u tag %08lx mask %04lx", addr, line, NLINES, tag, mask); Serial.println (sbuf);
    dcache_use++;

    // find set & free entry
    int free = -1;
    int set  = -1;
    for (unsigned s = 0; s < WAYS; s++) {
	word32 t = dlines[line].tag[s];
	if (t & FLAG_FREE)
	    free = s;
	else if ((t & ~mask) == tag) {
	    set  = s;
	    dcache_hit++;
	    break;
	}
    }


    // not found, free entry ?
    if (set < 0 && free < 0) {
	// cache eviction
	dcache_evicted++;

	// select set to drop
#if WAYS==2
	free = dlines[line].lru & 1;
	//printf ("addr %lx line %u evicted = %i with %lx\n", addr, line, free, dlines[line].tag[free]);
#else
#error "unsupported WAYS in dcache update LRU"
#endif

	// write back needed ?
	word32 tag = dlines[line].tag[free];
	if (tag & FLAG_DIRTY) {
	    dcache_wback++;
	    byte *p = &dcache_raw[line * WAYS * SRAM_PAGE + free * SRAM_PAGE];
	    ram_writePage (tag ^ FLAG_DIRTY, p);
	}
    }

    // cache fill
    if (set < 0) {
	set	= free;
	byte *p = &dcache_raw[line * WAYS * SRAM_PAGE + set * SRAM_PAGE];
	if (tag < sizeof (zROM)) {
	    for (unsigned i = tag; i < tag + SRAM_PAGE; i++)
		*p++ = pgm_read_byte_near (zROM + i);
	} else {
	    ram_readPage (tag, p);
	    //sprintf (sbuf, "\t| RAM load tag %lx", tag); Serial.println (sbuf);
	}
	dlines[line].tag[set] = tag;
	dcache_load++;
	//line_dump (line, "cache load");
    }

    // update LRU
#if WAYS == 2
    dlines[line].lru = set ? 0 : 1;
#else
#error "unsupported WAYS in dcache update LRU"
#endif

    // do operation
    byte *p = &dcache_raw[line * WAYS * SRAM_PAGE + set * SRAM_PAGE + (addr & mask)];
    if (write) {
	switch (bytes) {
	    case 1: *p = value; break;
	    case 2: p[0] = value >> 8; p[1] = value; break;
	    case 4: p[0] = value >> 24; p[1] = value >> 16; p[2] = value >> 8; p[3] = value; break;
	}
	dlines[line].tag[set] |= FLAG_DIRTY;
	//sprintf (sbuf, "\t| write %p (%p) line %u set %u diff %lu", p, dcache_raw, line, set, p - dcache_raw); Serial.println (sbuf);
	//line_dump (0, "ram write");
    } else {
	switch (bytes) {
	    case 1: value = *p; break;
	    case 2: value = (p[0] << 8) | p[1]; break;
	    case 4: value = (((word32) p[0]) << 24) | (((word32) p[1]) << 16) | (((word32) p[2]) << 8) | p[3]; break;
	}
    }
    return value;
}
