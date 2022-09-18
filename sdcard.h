#ifdef BOARD_ESP32
static SPIClass espi;
#else
#define espi SPI
#endif

#define SECTOR_BYTES	512
#define SD_KHZ		800

#define SDCARD_DEBUG

// card types
enum {
    SD_CARD_TYPE_SD1  = 1,
    /** standard capacity v2 sd card */
    SD_CARD_TYPE_SD2  = 2,
    /** high capacity sd card */
    SD_CARD_TYPE_SDHC = 3,
    /** extended capacity sd card */
    SD_CARD_TYPE_SDXC = 4
};

// SD operation timeouts
enum {
    SD_CMD0_RETRY    = 10,	// CMD0 retry count
    SD_CMD_TIMEOUT   = 300,	// command timeout ms
    SD_ERASE_TIMEOUT = 10000,	// erase timeout ms
    SD_INIT_TIMEOUT  = 2000,	// init timeout ms
    SD_READ_TIMEOUT  = 300,	// read timeout ms
    SD_WRITE_TIMEOUT = 600,	// write time out ms
};

// SD card states
enum {
    IDLE_STATE	= 0,
    READ_STATE	= 1,	// SD is in multi-sector read  state
    WRITE_STATE = 2,	// SD is in multi-sector write state
};

// errors
enum {
    SD_ERROR_NONE,

    SD_ERROR_ACMD41,
    SD_ERROR_ACMD51,
    SD_ERROR_CMD0,
    SD_ERROR_CMD8,
    SD_ERROR_CMD12,
    SD_ERROR_CMD18,
    SD_ERROR_CMD25,
    SD_ERROR_CMD58,
    SD_ERROR_CMD59,
    SD_ERROR_DMA,
    SD_ERROR_READ_CRC,
    SD_ERROR_READ_REG,
    SD_ERROR_READ_TIMEOUT,
    SD_ERROR_READ_TOKEN,
    SD_ERROR_STOP_TRAN,
    SD_ERROR_WRITE_DATA,
    SD_ERROR_WRITE_TIMEOUT
};

#define error(code)	sdst.errno = code

// SD responses & tokens
enum {
    R1_READY_STATE	 = 0x00,    // status for card in the ready state
    R1_IDLE_STATE	 = 0x01,    // status for card in the idle state
    R1_ILLEGAL_COMMAND	 = 0x04,    // status bit for illegal command
    DATA_START_SECTOR	 = 0xFE,    // start data token for read or write single sector
    STOP_TRAN_TOKEN	 = 0xFD,    // stop token for write multiple sectors
    WRITE_MULTIPLE_TOKEN = 0xFC,    // start data token for write multiple sectors
    DATA_RES_MASK	 = 0x1F,    // mask for data response tokens after a write sector operation
    DATA_RES_ACCEPTED	 = 0x05,    // write data accepted token
};

// sd card commands
enum {
    CMD0  = 0x00,   // GO_IDLE_STATE - init card in spi mode if CS low
    CMD8  = 0x08,   // SEND_IF_COND - verify SD Memory Card interface operating condition
    CMD9  = 0x09,   // SEND_CSD - read the Card Specific Data (CSD register)
    CMD10 = 0x0A,   // SEND_CID - read the card identification information (CID register)
    CMD12 = 0x0C,   // STOP_TRANSMISSION - end multiple sector read sequence
    CMD17 = 0x11,   // READ_SINGLE_SECTOR - read a single data sector from the card
    CMD18 = 0x12,   // READ_MULTIPLE_SECTOR - read multiple data sectors from the card
    CMD25 = 0x19,   // WRITE_MULTIPLE_SECTOR - write sectors of data until a STOP_TRANSMISSION
    CMD55 = 0x37,   // APP_CMD - escape for application specific command
    CMD58 = 0x3A,   // READ_OCR - read the OCR register of a card
    CMD59 = 0x3B,   // CRC_ON_OFF - enable or disable CRC checking

    // APP CMDS
    ACMD41 = 0x29,  // SD_SEND_OP_COMD - Sends host capacity support information and activates the card's initialization process
    ACMD51 = 0x33,  // Reads the SD Configuration Register (SCR)
};

// CRC 7 bits
static uint8_t CRC7 (const uint8_t* data, uint8_t n) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < n; i++) {
    uint8_t d = data[i];
    for (uint8_t j = 0; j < 8; j++) {
      crc <<= 1;
      if ((d & 0x80) ^ (crc & 0x80)) {
	crc ^= 0x09;
      }
      d <<= 1;
    }
  }
  return (crc << 1) | 1;
}

// Shift based CRC-CCITT. uses the x^16,x^12,x^5,x^1 polynomial
static uint16_t CRC_CCITT (const uint8_t* data, size_t n) {
    uint16_t crc = 0;
    for (size_t i = 0; i < n; i++) {
	crc = (uint8_t)(crc >> 8) | (crc << 8);
	crc ^= data[i];
	crc ^= (uint8_t)(crc & 0xff) >> 4;
	crc ^= crc << 12;
	crc ^= (crc & 0xff) << 5;
    }
    return crc;
}

// read high endian 4 bytes number
static uint32_t read4 (const uint8_t *p) {
    return (((uint32_t) p[0]) << 24) | (((uint32_t) p[1]) << 16)
	 | (((uint32_t) p[2]) <<  8) | (uint32_t) p[3]
	 ;
}

// sd card state
static struct {
    // operation conditions register
    uint32_t	ocr;

    // card identification
    struct {
	uint8_t     mid;	// manufacturer ID
	char	    oid[2];	// OEM/Application ID, ASCII
	char	    pname[5];	// product name, ASCII
	uint8_t     prev;	// product revision, 2 nibbles
	uint8_t     psn[4];	// product serial number
	uint8_t     mdt[2];	// manufacturing date, composed of 12 bits in RYYM Reserved Year Month, (offset from 2000)
	uint8_t     crc;	// CRC7 bits 1-7, bit 0 always 1
    }		cid;

    // union of all csd versions
    uint8_t csd[16];

    // scr register: bytes 0-3 SD Association, bytes 4-7 reserved for manufacturer
    uint8_t scr[8];

    // status
    uint8_t	    type;
    uint8_t	    errno;
    uint8_t	    state;
    bool	    active;

} sdst;

static int     sd_error () {return sdst.errno;}
static uint8_t sd_type ()  {return sdst.type;}

// delete ois single block allowed ?
static bool sd_eraseSingleBlock() {return sdst.csd[10] & 0x40 ? true : false;}

// return erase size in 512 byte blocks if eraseSingleBlock is false
static int sd_eraseSize() {return ((sdst.csd[10] & 0x3F) << 1 | sdst.csd[11] >> 7) + 1;}

// return true if the contents is copied or true if original
static bool sd_copy() {return sdst.csd[14] & 0x40 ? true : false;}

// return true if the entire card is permanently write protected
static bool sd_permWriteProtect() {return sdst.csd[14] & 0x20 ? true : false;}

// return true if the entire card is temporarily write protected
static bool sd_tempWriteProtect() {return sdst.csd[14] & 0x10 ? true : false;}

static uint32_t sd_capacity () {
    uint32_t c_size = 0;
    uint8_t	ver = sdst.csd[0] >> 6;
    if (ver == 0) {
	c_size	= (uint32_t)(sdst.csd[6] & 3) << 10;
	c_size |= (uint32_t) sdst.csd[7] << 2 | sdst.csd[8] >> 6;
	uint8_t c_size_mult = (sdst.csd[9] & 3) << 1 | sdst.csd[10] >> 7;
	uint8_t read_bl_len = sdst.csd[5] & 15;
	c_size = (c_size + 1) << (c_size_mult + read_bl_len + 2 - 9);
    } else if (ver == 1) {
	//Serial.println ("version 1");
	c_size	= (uint32_t)(sdst.csd[7] & 63) << 16;
	c_size |= (uint32_t) sdst.csd[8] << 8;
	c_size |= sdst.csd[9];
	c_size	= (c_size + 1) << 10;
    }
    return c_size;
}

// return false if all zero, true if all one.
static bool sd_dataAfterErase () {return 0x80 & sdst.scr[1];}

static uint8_t sd_spec () {return sdst.scr[0] & 0xF;}
static bool    sd_spec3() {return sdst.scr[2] & 0x80;}
static bool    sd_spec4() {return sdst.scr[2] & 0x4;}
static uint8_t sd_specX() {return (sdst.scr[2] & 0x3) << 2 | sdst.scr[3] >> 6;}

static int16_t sd_SpecVer() {
    if (sd_spec() > 2) {
	return -1;
    } else if (sd_spec() < 2) {
	return sd_spec() ? 110 : 101;
    } else if (!sd_spec3()) {
	return 200;
    } else if (!sd_spec4() && !sd_specX()) {
	return 300;
    }
    return 400 + 100 * sd_specX ();
}

static void dump_ocr () {
    uint32_t ocr = sdst.ocr;
    sprintf (sbuf, "Available voltage ranges: OCR %08lx", (unsigned long) ocr); Serial.println (sbuf);

    #define BIT_CAPTION(bit,title)    if (ocr & (1ul << bit)) Serial.println ("\t" title);
    BIT_CAPTION ( 4, "1.6-1.7");
    BIT_CAPTION ( 5, "1.7-1.8");
    BIT_CAPTION ( 6, "1.8-1.9");
    BIT_CAPTION ( 7, "1.9-2.0");
    BIT_CAPTION ( 8, "2.0-2.1");
    BIT_CAPTION ( 9, "2.1-2.2");
    BIT_CAPTION (10, "2.2-2.3");
    BIT_CAPTION (11, "2.3-2.4");
    BIT_CAPTION (12, "2.4-2.5");
    BIT_CAPTION (13, "2.5-2.6");
    BIT_CAPTION (14, "2.6-2.7");
    BIT_CAPTION (15, "2.7-2.8");
    BIT_CAPTION (16, "2.8-2.9");
    BIT_CAPTION (17, "2.9-3.0");
    BIT_CAPTION (18, "3.0-3.1");
    BIT_CAPTION (19, "3.1-3.2");
    BIT_CAPTION (20, "3.2-3.3");
    BIT_CAPTION (21, "3.3-3.4");
    BIT_CAPTION (22, "3.4-3.5");
    BIT_CAPTION (23, "3.5-3.6");
    BIT_CAPTION (30, "High Capacity Card SDHC (> 2GB)");
    BIT_CAPTION (31, "Card power up status bit (power up busy, active low)");
}

static void dump_cid () {
    sprintf (sbuf, "Manufacturer ID: %02X", sdst.cid.mid); Serial.println (sbuf);
    sprintf (sbuf, "OEM ID: %c%c", sdst.cid.oid[0], sdst.cid.oid[1]); Serial.println (sbuf);
    sprintf (sbuf, "Product: %c%c%c%c%c",
	sdst.cid.pname[0], sdst.cid.pname[1], sdst.cid.pname[2],
	sdst.cid.pname[3], sdst.cid.pname[4]
	); Serial.println (sbuf);
    sprintf (sbuf, "Revision: %u.%u", sdst.cid.prev >> 4, sdst.cid.prev & 0xf); Serial.println (sbuf);
    sprintf (sbuf, "Serial number: %08lx", (unsigned long) read4 (sdst.cid.psn)); Serial.println (sbuf);
    sprintf (sbuf, "Manufacturing date: %02u/%04u",
	sdst.cid.mdt[1] & 0x0f,
	2000 + ((sdst.cid.mdt[0] & 0x0f) << 4) + (sdst.cid.mdt[1] >> 4)
	); Serial.println (sbuf);
    sprintf (sbuf, "CRC7: %02x (= %02x)", sdst.cid.crc, CRC7 (&sdst.cid.mid, 15)); Serial.println (sbuf);
}

static void dump_csd () {
    sprintf (sbuf, "cardSize: %lu * 512 sectors", (long) sd_capacity ()); Serial.println (sbuf);
    sprintf (sbuf, "flash erase size: %i blocks", sd_eraseSize ()); Serial.println (sbuf);
    sprintf (sbuf, "eraseSingleBlock: %s", sd_eraseSingleBlock () ? "true" : "false"); Serial.println (sbuf);
    sprintf (sbuf, "copy: %s", sd_copy () ? "true" : "false"); Serial.println (sbuf);
    sprintf (sbuf, "tempWriteProtected: %s", sd_tempWriteProtect () ? "true" : "false"); Serial.println (sbuf);
    sprintf (sbuf, "permWriteProtected: %s", sd_permWriteProtect () ? "true" : "false"); Serial.println (sbuf);
    sprintf (sbuf, "CRC7: %02x (= %02x)", sdst.csd[15], CRC7 (sdst.csd, 15)); Serial.println (sbuf);
}

static void dump_scr () {
    Serial.print ("Card type: ");
    switch (sd_type ()) {
	case SD_CARD_TYPE_SD1:
	    Serial.println ("SD1");
	    break;
	case SD_CARD_TYPE_SD2:
	    Serial.println ("SD2");
	    break;
	case SD_CARD_TYPE_SDHC:
	    if (sd_capacity() < 70000000ul)
		Serial.println ("SDHC");
	    else
		Serial.println ("SDXC");
	    break;
	default:
	    Serial.println ("Unknown");
    }
    sprintf (sbuf, "sdSpecVer: %04x", sd_SpecVer ()); Serial.println (sbuf);
    sprintf (sbuf, "dataAfterErase: %s", sd_dataAfterErase() ? "true" : "false"); Serial.println (sbuf);

    // determine highspeedmode
    //static uint8_t cmd6Data[64];
    //bool hsm = sd_SpecVer() && sd.cardCMD6 (0x00FFFFFF, cmd6Data) && (2 & cmd6Data[13]);
    //sprintf (sbuf, "HighSpeedMode: %s\n", hsm ? "true" : "false"); Serial.println (sbuf);
}

typedef uint32_t timeout_t;

static uint32_t  millis32 () {return millis();}
static timeout_t timeout  (uint32_t ms) {return ms + millis32 ();}
static bool	 timedOut (timeout_t t) {return (int32_t) (t - millis32()) < 0;}

static void spiUnselect() {
    digitalWrite (SS, HIGH);
}

static void spiStop () {
    if (sdst.active) {
	sdst.active = false;
	spiUnselect ();

	// Insure MISO goes to low Z.
	espi.transfer (0xFF);
	espi.endTransaction ();
    }
}

static void spiStart () {
    if (!sdst.active) {
	sdst.active = true;
	espi.beginTransaction (SPISettings (SD_KHZ * 1000, MSBFIRST, SPI_MODE0));
	digitalWrite (SS, LOW);

	// Dummy byte to drive MISO busy status.
	espi.transfer (0xFF);
    }
}

static void spiSend (const uint8_t* buf , size_t count) {
    for (size_t i = 0; i < count; i++)
	espi.transfer (buf[i]);
}

static uint8_t spiReceive () {return espi.transfer (0xFF);}

static uint8_t spiReceiveBuf (uint8_t* buf, size_t count) {
    for (size_t i = 0; i < count; i++)
	buf[i] = espi.transfer (0xff);
    return 0;
}

static bool sd_waitReady(uint16_t ms) {
    timeout_t to = timeout (ms);
    while (spiReceive() != 0xFF) {
	if (timedOut (to)) {
	    return false;
	}
    }
    return true;
}

static uint8_t sd_command (uint8_t cmd, uint32_t arg);

static bool sd_readStop () {
    sdst.state = IDLE_STATE;
    if (sd_command (CMD12, 0)) {
	error (SD_ERROR_CMD12);
	goto fail;
    }
    spiStop ();
    return true;

fail:
    spiStop ();
    return false;
}

static bool sd_writeStop () {
    if (!sd_waitReady (SD_WRITE_TIMEOUT))
	goto fail;
    espi.transfer (STOP_TRAN_TOKEN);
    spiStop ();
    sdst.state = IDLE_STATE;
    return true;
fail:
    error (SD_ERROR_STOP_TRAN);
    spiStop();
    return false;
}

static bool sd_readStart (uint32_t sector) {
    if (sdst.type < SD_CARD_TYPE_SDHC) {
	sector <<= 9;
    }
    byte resp = sd_command (CMD18, sector);
    if (resp) {
	sprintf (sbuf, "CMD18 resp is %02x", resp); Serial.println (sbuf);
	error (SD_ERROR_CMD18);
	goto fail;
    }
    sdst.state = READ_STATE;
    return true;
fail:
    spiStop ();
    return false;
}

static bool sd_sync () {
    if (sdst.state == WRITE_STATE) {
	return sd_writeStop ();
    }
    if (sdst.state == READ_STATE) {
	return sd_readStop ();
    }
    return true;
}

static uint8_t sd_command (uint8_t cmd, uint32_t arg) {
    //sprintf (sbuf, "cardcommand %02x %02x", cmd, arg); Serial.println (sbuf);

    // sync
    if (!sd_sync ())
	return 0xFF;

    // select card
    if (!sdst.active)
	spiStart ();

    // wait ready
    if (cmd != CMD0 && cmd != CMD12 && !sd_waitReady (SD_CMD_TIMEOUT))
	return 0xff;

    // form message
    uint8_t buf[6];
    buf[0] = 0x40U | cmd;
    buf[1] = arg >> 24;
    buf[2] = arg >> 16;
    buf[3] = arg >>  8;
    buf[4] = arg;
    buf[5] = CRC7 (buf, 5);
    //sprintf (sbuf, "\tCMD %02x %02x %02x %02x %02x | %02x", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]); Serial.println (sbuf);

    // send message
    spiSend (buf, 6);

    // discard first fill byte to avoid MISO pull-up problem.
    spiReceive ();

    // there are 1-8 fill bytes before response.  fill bytes should be 0XFF.
    uint16_t n = 0;
    uint8_t sb = 0;
    do {
	sb = spiReceive ();
    } while ((sb & 0x80) && ++n < 10);
    //sprintf (sbuf, "\tresponse = %x (%u)", sb, n); Serial.println (sbuf);

    // done
    return sb;
}

static uint8_t sd_appcmd (uint8_t cmd, uint32_t arg) {
    sd_command (CMD55, 0);
    return sd_command (cmd, arg);
}

static bool sd_readData (uint8_t* dst, size_t count) {
    // wait for start sector token
    timeout_t to = timeout (SD_READ_TIMEOUT);
    uint8_t resp = 0;
    while ((resp = spiReceive()) == 0xFF) {
	if (timedOut (to)) {
	    error (SD_ERROR_READ_TIMEOUT);
	    spiStop ();
	    return false;
	}
    }
    if (resp != DATA_START_SECTOR) {
	error (SD_ERROR_READ_TOKEN);
	spiStop ();
	return false;
    }

    // transfer data
    if ((resp = spiReceiveBuf (dst, count))) {
	error (SD_ERROR_DMA);
	spiStop ();
	return false;
    }

    // get crc
    uint16_t  hi = spiReceive ();
    uint16_t  lo = spiReceive ();
    uint16_t crc = (hi << 8) | lo;
    if (crc != CRC_CCITT (dst, count)) {
	Serial.println ("CRC fail !");
	error (SD_ERROR_READ_CRC);
	spiStop ();
	return false;
    }

    // done
    return true;
}

static bool sd_readRegister (uint8_t cmd, void *buf) {
    if (sd_command (cmd, 0)) {
	error (SD_ERROR_READ_REG);
	goto fail;
    }

    // read body
    if (!sd_readData ((byte *) buf, 16))
	goto fail;
    //bdump ((byte *) buf, 16);

    // done
    spiStop();
    return true;

    // fail
fail:
    spiStop();
    return false;
}

static bool sd_readCID (void) {return sd_readRegister (CMD10, &sdst.cid);}
static bool sd_readCSD (void) {return sd_readRegister (CMD9,  &sdst.csd);}

static bool sd_readSCR (void) {
    if (sd_appcmd (ACMD51, 0)) {
	error (SD_ERROR_ACMD51);
	goto fail;
    }
    if (!sd_readData (sdst.scr, sizeof (sdst.scr))) {
	goto fail;
    }
    spiStop();
    return true;
fail:
    spiStop();
    return false;
}

static bool sd_readSectors (uint32_t sector, uint8_t* dst, size_t ns) {
    if (sdst.state != READ_STATE && !sd_readStart (sector))
	goto fail;

    for (size_t i = 0; i < ns; i++, dst += 512) {
	if (!sd_readData (dst, 512)) {
	    goto fail;
	}
    }
    return true; //m_dedicatedSpi ? true : readStop ();
fail:
    return false;
}

static bool sd_writeStart (uint32_t sector) {
    // use address if not SDHC card
    if (sdst.type != SD_CARD_TYPE_SDHC)
	sector <<= 9;

    if (sd_command (CMD25, sector)) {
	error (SD_ERROR_CMD25);
	goto fail;
    }
    sdst.state = WRITE_STATE;
    return true;
fail:
    spiStop();
    return false;
}

static bool sd_writeData (uint8_t token, const uint8_t* src) {
    uint16_t crc = CRC_CCITT (src, 512);
    espi.transfer (token);
    spiSend	 (src, 512);
    espi.transfer (crc >> 8);
    espi.transfer (crc & 0xFF);

    byte status = spiReceive ();
    if ((status & DATA_RES_MASK) != DATA_RES_ACCEPTED) {
	error (SD_ERROR_WRITE_DATA);
	goto fail;
    }
    return true;
fail:
    spiStop ();
    return false;
}

static bool sd_writeWait (const uint8_t* src) {
    // wait for previous write to finish
    if (!sd_waitReady (SD_WRITE_TIMEOUT)) {
	error (SD_ERROR_WRITE_TIMEOUT);
	goto fail;
    }
    if (!sd_writeData (WRITE_MULTIPLE_TOKEN, src))
	goto fail;
    return true;
fail:
    spiStop();
    return false;
}

static bool sd_writeSectors (uint32_t sector, const uint8_t* src, size_t ns) {
    if (!sd_writeStart (sector))
	goto fail;
    for (size_t i = 0; i < ns; i++, src += 512) {
	if (!sd_writeWait (src))
	    goto fail;
    }
    return sd_writeStop ();
fail:
    spiStop();
    return false;
}

/*
 * detect
 */
static bool sd_detect (uint32_t Hz) {
    // spi setup
#ifdef BOARD_ESP32
    espi = SPIClass (VSPI);
#endif
    espi.begin ();
    pinMode	 (SS, OUTPUT);
    digitalWrite (SS, HIGH);

    // init state
    memset (&sdst, 0, sizeof (sdst));

    // locals
    timeout_t to = 0;
    uint32_t arg = 0;
    byte     sic = 0;

    // preamble clock cycles with CS active.
    spiStart ();
    for (uint8_t i = 0; i < 20; i++)
	espi.transfer (0xff);

    // reset command to enter idle state in SPI mode
    for (uint8_t i = 1;; i++) {
	if (sd_command (CMD0, 0) == R1_IDLE_STATE)
	    break;
	if (i == SD_CMD0_RETRY) {
	    error (SD_ERROR_CMD0);
	    goto fail;
	}

	// force any active transfer to end for an already initialized card.
	for (uint8_t j = 0; j < 0xff; j++)
	    espi.transfer (0xff);
    }
#ifdef SDCARD_DEBUG
    Serial.println ("SD card reset done");
#endif

    // enable CRC checking
    if (sd_command (CMD59, 1) != R1_IDLE_STATE) {
	error (SD_ERROR_CMD59);
	goto fail;
    }
#ifdef SDCARD_DEBUG
    Serial.println ("Enabled CRC checking");
#endif

    // check SD version (SEND_IF_COND) card
    sic = sd_command (CMD8, 0x1AA);
    if (!(sic & R1_ILLEGAL_COMMAND)) {
	sdst.type = SD_CARD_TYPE_SD2;

	// response must echo arg
	for (uint8_t i = 0; i < 4; i++)
	    sic = spiReceive ();
	if (sic != 0xaa) {
	    error (SD_ERROR_CMD8);
	    goto fail;
	}
    } else
	sdst.type = SD_CARD_TYPE_SD1;
#ifdef SDCARD_DEBUG
    sprintf (sbuf, "Card type done: %i", sdst.type); Serial.println (sbuf);
#endif

    // SD_SEND_OP_COND: initialize card and send host supports SDHC if SD2
    arg = sdst.type == SD_CARD_TYPE_SD2 ? 0x40000000 : 0;
    to = timeout (SD_INIT_TIMEOUT);
    while (sd_appcmd (ACMD41, arg) != R1_READY_STATE) {
	// check for timeout
	if (timedOut (to)) {
#ifdef SDCARD_DEBUG
	    Serial.println ("sd card init timeout ACMD41");
#endif
	    error (SD_ERROR_ACMD41);
	    goto fail;
	}
    }
#ifdef SDCARD_DEBUG
    Serial.println ("SD_SEND_OP_COND done");
#endif

    // if SD2 read OCR register to check for SDHC card
    if (sdst.type == SD_CARD_TYPE_SD2) {
	// read OCR register
	if (sd_command (CMD58, 0)) {
	    error (SD_ERROR_CMD58);
	    goto fail;
	}

	// data
	uint8_t ocr[4];
	for (int i = 0; i < 4; i++)
	    ocr[i] = spiReceive ();
	sdst.ocr = read4 (ocr);
	//dump_ocr ();

	// check if SDHC card
	if ((ocr[0] & 0xC0) == 0xC0) {
	    sdst.type = SD_CARD_TYPE_SDHC;
#ifdef SDCARD_DEBUG
	    Serial.println ("SDHC Card");
#endif
	}
    }

    // done
    spiStop ();
    return true;
fail:
    spiStop ();
    return false;
}
