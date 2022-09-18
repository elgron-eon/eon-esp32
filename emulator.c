/*
 * emulator.c
 *
 * eonduino emulator
 * (c) JCGV, junio del 2022
 *
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <poll.h>
#include <sys/time.h>

/*
 * terminal support
 */
static struct termios term;

static void term_done (void) {
    tcsetattr (STDIN_FILENO, TCSANOW, &term);
}

static void term_setup (void) {
    tcgetattr (STDIN_FILENO, &term);
    struct termios t = term;
    t.c_lflag &= ~ICANON;
    t.c_lflag &= ~ECHO;
    tcsetattr (STDIN_FILENO, TCSANOW, &t);
    atexit (term_done);
}

/*
 * disk image support
 */
static int	disk_fd;
static unsigned disk_nsect;
static uint8_t	disk_buf[512];

static void disk_setup (void) {
    static const char path[] = "/tmp/sdcard.disk";
    disk_fd = open (path, O_RDWR);
    if (disk_fd < 0) {
	fprintf (stderr, "emulator: can not open image [%s]: %m\n", path);
	exit (1);
    }

    off_t bytes = lseek (disk_fd, 0, SEEK_END);
    disk_nsect	= bytes / 512;
    printf ("\t| emulator with %u disk sectors\n", disk_nsect);
}

static void disk_read (unsigned bno) {
    off_t off = bno * sizeof (disk_buf);
    printf ("\t| DISK read #%u at %lu\n", bno, off);
    if (lseek (disk_fd, off, SEEK_SET) < 0
    ||	read  (disk_fd, disk_buf, sizeof (disk_buf)) != sizeof (disk_buf)) {
	printf ("\t| disk block #%u read error: %m\n", bno);
    }
}

static void disk_write (unsigned bno) {
    off_t off = bno * sizeof (disk_buf);
    printf ("\t| DISK write #%u at %lu\n", bno, off);
    if (lseek (disk_fd, off, SEEK_SET) < 0
    ||	write (disk_fd, disk_buf, sizeof (disk_buf)) != sizeof (disk_buf)) {
	printf ("\t| disk block #%u write error: %m\n", bno);
    }
}

/*
 * support code
 */
typedef uint8_t byte;

#define PROGMEM
#define F(str)	    str
#define HIGH	    1
#define LOW	    0
#define SEXT8(n)    LIMIT32 ((long) (int8_t ) (n))
#define SEXT16(n)   LIMIT32 ((long) (int16_t) (n))
#define LIMIT32(n)  ((n) & 0x0fffffffful)

// ROM read support
static byte pgm_read_byte_near (const byte *p) {return *p;}

// PINS
enum {OUTPUT, INPUT_PULLUP};

static byte vled[64];
static void pinMode	 (unsigned pin, unsigned mode) {}
//static int  digitalRead  (unsigned pin) {return HIGH;}
static void digitalWrite (unsigned pin, unsigned v) {vled[pin] = v;}

// idle
static void idle (void) {
#if 0
    // wait for event
    struct pollfd pfd = {
	.fd	 = STDIN_FILENO,
	.events  = POLLIN,
	.revents = 0
    };
    poll (&pfd, 1, -1);
#else
    struct timeval tv;
    tv.tv_sec  = 0;
    tv.tv_usec = 50 * 1000;
    select (0, NULL, NULL, NULL, &tv);
#endif
}

// serial console
static void s_print (const char *msg) {
    printf ("%s", msg);
    fflush (stdout);
}

static void s_println (const char *msg) {
    printf ("%s\n", msg);
    fflush (stdout);
}

static unsigned millis (void) {
    static bool     ready;
    static uint64_t ms0;

    struct timeval tv;
    gettimeofday (&tv, NULL);
    uint64_t now = tv.tv_sec * 1000 + tv.tv_usec / 1000;

    unsigned ms = 0;
    if (ready) {
	ms = now - ms0;
    } else {
	ms0   = now;
	ready = true;
    }
    return ms;
}

static void delay   (unsigned ms) {}
static void s_write (unsigned c) {fputc (c, stdout); fflush (stdout);}
static void s_flush (void) {fflush (stdout);}
static void s_begin (unsigned b) {}
static bool s_ready (void) {return true;}

static bool s_available (void) {
    struct pollfd pfd = {
	.fd	 = STDIN_FILENO,
	.events  = POLLIN,
	.revents = 0
    };
    int rc = poll (&pfd, 1, 0);
    return rc > 0;
}

static byte s_read (void) {
    byte data = 0;
    read (STDIN_FILENO, &data, 1);
    return data;
}

static struct {
    void (*print) (const char *);
    void (*println) (const char *);
    void (*begin) (unsigned);
    void (*flush) (void);
    void (*write) (unsigned);
    bool (*availableForWrite) (void);
    bool (*available) (void);
    byte (*read)  (void);
} Serial = {s_print, s_println, s_begin, s_flush, s_write, s_ready, s_available, s_read};

// WIRE
static uint8_t iow, ior, iomode;
static uint8_t iobuf[32];
enum {IO_READ, IO_WRITE};

static void w_begin (void) {}
static bool w_available (void) {return true;}

static void w_beginT (unsigned port) {
    iobuf[0] = port;
    iow      = 1;
    iomode   = IO_WRITE;
}

static int w_request (unsigned port, unsigned v) {
    ior    = 0;
    iomode = IO_READ;
    //printf ("wire request %02x %02x\n", port, v);
    return v;
}

static int w_end (void);

static void w_write (unsigned reg) {
    iobuf[iow++] = reg;
    //printf ("write %02x at %i\n", reg, iow);
}

static unsigned w_read (void) {
    unsigned v = iobuf[ior++];
    //printf ("read %02x at %i\n", v, ior);
    return v;
}

static struct {
    void     (*beginTransmission) (unsigned port);
    int      (*requestFrom)	  (unsigned port, unsigned);
    int      (*endTransmission)   (void);
    void     (*write)		  (unsigned r);
    unsigned (*read)		  (void);
    bool     (*available)	  (void);
    void     (*begin)		  (void);
} Wire = {w_beginT, w_request, w_end, w_write, w_read, w_available, w_begin};

typedef struct RTCDateTime RTCDateTime;

// SPI
#define MSBFIRST    0
#define SPI_MODE0   0
#define SS	    53
static int SPISettings (unsigned mhz, unsigned msb, unsigned mode) {return 0;}

static unsigned spi_st;

static void i_begin (void) {}
static void i_tran  (int)  {spi_st = 0;}
static void i_end   (void) {}

static unsigned i_transfer (unsigned byte);

static struct {
    void	(*begin) (void);
    void	(*beginTransaction) (int);
    void	(*endTransaction) (void);
    unsigned	(*transfer) (unsigned);
} SPI = {i_begin, i_tran, i_end, i_transfer};

/*
 * ino code
 */
#define EMULATOR
#include "main.ino"

void no_warn_unused (void) {
    dump_scr ();
    dump_cid ();
    dump_csd ();
    dump_ocr ();
    sd_readSCR ();
    sd_readCID ();
    sd_readSectors  (0, 0, 0);
    sd_writeSectors (0, 0, 0);
}

/*
 * i/o support
 */
static int w_end (void) {
    if (iomode == IO_WRITE) {
	unsigned port = iobuf[0];
	switch (port) {
	    case DS3231_ADDRESS:
		switch (iobuf[1]) {
		    case DS3231_REG_CONTROL:
			iobuf[0] = 0x00;
			break;
		    case DS3231_REG_TIME: {
			    time_t t = time (NULL);
			    struct tm tm;
			    gmtime_r (&t, &tm);
			    iobuf[6] = dec2bcd (tm.tm_year + 1900 - 2000);
			    iobuf[5] = dec2bcd (tm.tm_mon + 1);
			    iobuf[4] = dec2bcd (tm.tm_mday);
			    iobuf[3] = tm.tm_wday;
			    iobuf[2] = dec2bcd (tm.tm_hour);
			    iobuf[1] = dec2bcd (tm.tm_min);
			    iobuf[0] = dec2bcd (tm.tm_sec);
			} break;
		}
		break;
	    default:
		printf ("I/O write port %02x:", port);
		for (int i = 1; i < iow; i++)
		    printf (" %02X", iobuf[i]);
		printf ("\n");
		exit   (1);
		break;
	}
    } else {
	//printf ("end\n");
    }
    return 0;
}

static void build_csd (uint8_t *p) {
    p[0] = 0x40;    // version 1

    unsigned n = (disk_nsect >> 10) - 1;
    p[7] = n >> 16;
    p[8] = n >> 8;
    p[9] = n;
}

static unsigned i_transfer (unsigned byte) {
    static unsigned spi_st2, spi_arg, crc;
    static uint8_t spi_reg[16];

    bool resp = false;
    switch (spi_st) {
	case 0: // sdcard cmd
	    switch (byte) {
		case 0xff:  // ignore fill bytes
		    break;
		case 0x40:  // cmd0
		case 0x7b:  // cmd59
		case 0x77:  // appcmd
		    spi_st  = 1;
		    spi_arg = 0;
		    spi_st2 = 6;
		    break;
		case 0x48:  // cmd8
		    spi_st  = 1;
		    spi_arg = 0;
		    spi_st2 = 8;
		    break;
		case 0x4c:  // cmd12
		case 0x69:  // ACMD41
		    spi_st  = 1;
		    spi_arg = 0;
		    spi_st2 = 14;
		    break;
		case 0x7a:  // cmd58
		    spi_st  = 1;
		    spi_arg = 0;
		    spi_st2 = 15;
		    break;
		case 0x49:  // cmd9, read CSD
		    spi_st  = 1;
		    spi_arg = 0;
		    spi_st2 = 16;
		    break;
		case 0x52:  // cmd18, read multiple sector
		    spi_st  = 1;
		    spi_arg = 0;
		    spi_st2 = 22;
		    break;
		case 0x59:  // cmd25, write multiple sector
		    spi_st  = 1;
		    spi_arg = 0;
		    spi_st2 = 26;
		    break;
		default:
		    printf ("SPI cmd unknown %02x\n", byte);
		    exit   (1);
		    break;
	    }
	    break;
	case 1: // arg high byte
	    spi_arg = byte;
	    spi_st  = 2;
	    break;
	case 2: // arg middle high byte
	    spi_arg = (spi_arg << 8) | byte;
	    spi_st  = 3;
	    break;
	case 3: // arg middle low byte
	    spi_arg = (spi_arg << 8) | byte;
	    spi_st  = 4;
	    break;
	case 4: // arg low byte
	    spi_arg = (spi_arg << 8) | byte;
	    spi_st  = 5;
	    break;
	case 5: // ignore crc
	    spi_st = spi_st2;
	    break;
	case 6: // cmd0 | cmd59
	    spi_arg = R1_IDLE_STATE;
	    spi_st  = 7;
	    break;
	case 7: // respond single byte
	    resp   = true;
	    byte   = spi_arg;
	    spi_st = 0;
	    break;
	case 8: // cmd8
	    spi_st = 9;
	    break;
	case 9: // 0 status
	    resp   = true;
	    byte   = 0;
	    spi_st = 10;
	    break;
	case 10: // echo arg 1/4
	    resp   = true;
	    byte   = spi_arg >> 24;
	    spi_st = 11;
	    break;
	case 11: // echo arg 2/4
	    resp   = true;
	    byte   = spi_arg >> 16;
	    spi_st = 12;
	    break;
	case 12: // echo arg 3/4
	    resp   = true;
	    byte   = spi_arg >> 8;
	    spi_st = 13;
	    break;
	case 13: // echo arg 4/4
	    resp   = true;
	    byte   = spi_arg;
	    spi_st = 0;
	    break;
	case 14: // acmd41
	    spi_arg = R1_READY_STATE;
	    spi_st  = 7;
	    break;
	case 15: // cmd58, send OCR
	    spi_st  = 9;
	    spi_arg = 0xc0FFFF00;
	    break;
	case 16: // cmd9, read CSD
	    build_csd (spi_reg);
	    spi_arg = 0;
	    spi_st  = 17;
	    crc     = CRC_CCITT (spi_reg, 16);
	    break;
	case 17: // register read response
	    resp   = true;
	    byte   = 0;
	    spi_st = 18;
	    break;
	case 18: // data start
	    resp   = true;
	    byte   = DATA_START_SECTOR;
	    spi_st = 19;
	    break;
	case 19: // send register
	    resp = true;
	    byte = spi_reg[spi_arg++];
	    if (spi_arg >= 16) spi_st = 20;
	    break;
	case 20: // crc high byte
	    resp   = true;
	    byte   = crc >> 8;
	    spi_st = 21;
	    break;
	case 21: // crc low byte
	    resp   = true;
	    byte   = crc;
	    spi_st = 0;
	    break;
	case 22: // read sectors
	    spi_st = 23;
	    break;
	case 23: // data start token
	    resp   = true;
	    byte   = 0;
	    spi_st = 24;
	    break;
	case 24: // data start token
	    disk_read (spi_arg);
	    resp    = true;
	    byte    = 0xfe;
	    spi_arg = 0;
	    spi_st  = 25;
	    break;
	case 25: // sector body
	    resp = true;
	    byte = disk_buf[spi_arg++];
	    if (spi_arg >= 512) {
		spi_st = 20;
		crc    = CRC_CCITT (disk_buf, sizeof (disk_buf));
	    }
	    break;
	case 26: // write sectors
	    spi_st = 27;
	    break;
	case 27: // accept request
	    resp   = true;
	    byte   = 0;
	    crc    = 0;
	    spi_st = 28;
	    break;
	case 28: // wait for token
	    if (byte == 0xfc)
		spi_st = 29;
	    break;
	case 29: // sector body
	    disk_buf[crc++] = byte;
	    if (crc >= 512) spi_st = 30;
	    break;
	case 30:    // ignore crc
	    spi_st = 31;
	    break;
	case 31:    // ignore crc
	    spi_st = 32;
	    break;
	case 32:    // send ack
	    disk_write (spi_arg);
	    spi_st = 33;
	    resp   = true;
	    byte   = 0x05;
	    break;
	case 33:    // receive stop token
	    if (byte == 0xfd)
		spi_st = 0;
	    break;
    }
    return resp ? byte : 0xff;
}

/*
 * entry point
 */
int main (int argc, char **argv) {
    // setup
    term_setup ();
    disk_setup ();
    setup ();

    // loop
    while (eon.st != S_DONE)
	loop ();

    // report
    if (vled[zBAD]) {
	printf ("fail led at pc %08lx status %04lx:", eon.pc, eon.s[REG_STATUS]);
	for (unsigned i = 0; i < 16; i++)
	    printf ("%sR%02u=%08x.%08x", i % 4 == 0 ? "\n\t" : " ", i, (uint32_t) (eon.r[i] >> 32), (uint32_t) eon.r[i]);
	printf ("\n");

	// dump line 0
	line_dump ( 0, "Zero Line");
	line_dump (58, "SP   Line");
	line_dump (63, "RSP  Line");

	// dump compiled word
	unsigned begin = 0x4000;
	unsigned end   = begin + 128;
	for (; begin < end; begin += 16) {
	    // get data
	    uint8_t s[16];

	    // print offset & raw bytes
	    printf ("%04x ", begin);
	    for (unsigned i = 0; i < 16; ++i) {
		s[i] = dcache (begin + i, false, 1, 0);
		printf (" %02x", s[i]);
	    }

	    // print ascii
	    printf ("  ");
	    for (unsigned i = 0; i < 16; ++i) {
		int d = s[i];
		printf ("%c", d > 31 && d < 127 ? d : '.');
	    }
	    printf ("\n");
	}
	return 0;

	// dump cache status
	for (unsigned i = 0; i < NLINES; i++) {
	    char title[8];
	    sprintf (title, "L#%02x", i);
	    line_dump (i, title);
	}
    }
    return 0;
}
