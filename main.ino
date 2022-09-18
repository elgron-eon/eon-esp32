/*
 * main.ino
 *
 * eon computer with esp8266
 * (c) JCGV, septiembre del 2022
 *
 */
#ifndef EMULATOR
#include <Wire.h>
#include <SPI.h>
#endif

// config
#define VERSION     "0.0.0"
#define DELAYMS     20
#define RESETCOUNT  15

// emulator compatibility macros
#ifndef LIMIT32
#define LIMIT32(n)  n
#endif
#ifndef SEXT8
#define SEXT8(n)    ((int32_t) (int8_t) (n))
#endif
#ifndef SEXT16
#define SEXT16(n)   ((int32_t) (int16_t) (n))
#endif

// pins used
#ifdef BOARD_ESP32

#elif defined (BOARD_ESP8266)

// nodemcv2 esp8266
#define zRESET	D5
#define zBAD	D6
#define zRST	D7
#define zRUN	LED_BUILTIN

#else

// emulator build
#define zRESET	13
#define zBAD	12
#define zRST	11
#define zRUN	10

#endif

// basic types
typedef unsigned int  word16;
typedef unsigned long word32;

// output format buffer
static char sbuf[128];

// ROM
#include "rom.h"

// dcache
#include "dcache.h"

// real time clock
#include "rtc.h"

// sdcard
#include "sdcard.h"

// cpu
static word32 cpu_in  (word32 port);
static void   cpu_out (word32 port, word32 v);

#ifndef EMULATOR
static void idle () {}
#endif
#include "cpu.h"

// I/O support
static byte	rtc_iobuf[8];
static byte	rtc_ionow;
static bool	disk_ready;
static byte	disk_cmd;
static uint16_t disk_addr;
static word32	timer, timer_inc;

static word32 cpu_in (word32 port) {
    word32 v = 0;
    switch (port) {
	case 0x00:
	    // console write ready
	    v = Serial.availableForWrite () > 0 ? 1 : 0;
	    break;
	case 0x01:
	    // console read (after IRQ signal)
	    v = Serial.read () & 0x7F;
	    break;
	case 0x02:
	    // rtc control port ready
	    v = 1;
	    break;
	case 0x03:
	    // read RTC data
	    v = rtc_iobuf[rtc_ionow++];
	    break;
	case 0x10:
	    // disk ctrl ready
	    v = 1;
	    break;
	default:
	    sprintf (sbuf, "\t| IN %lx = %04lx\n", port, v);
	    Serial.print (sbuf);
	    break;
    }
    return v;
}

static void cpu_out (word32 port, word32 v) {
    switch (port) {
	case 0x01:
	    Serial.write ((byte) v);
	    break;
	case 0x03:
	    // rtc command (0 = read, 1 = write)
	    if (v == 0) {
		RTCDateTime dt = rtc_getDateTime ();
		rtc_iobuf[0] = dt.year >> 8;
		rtc_iobuf[1] = dt.year;
		rtc_iobuf[2] = dt.month;
		rtc_iobuf[3] = dt.day;
		rtc_iobuf[4] = dt.dayOfWeek;
		rtc_iobuf[5] = dt.hour;
		rtc_iobuf[6] = dt.minute;
		rtc_iobuf[7] = dt.second;
		rtc_ionow    = 0;
		eon_irq (IRQ_RTC);
	    } else {
		rtc_ionow = 0;
	    }
	    break;
	case 0x04:
	    // rtc new data
	    rtc_iobuf[rtc_ionow++] = v;
	    if (rtc_ionow == 8) {
		RTCDateTime dt;
		dt.year      = (rtc_iobuf[0] << 8) | rtc_iobuf[1];
		dt.month     = rtc_iobuf[2];
		dt.day	     = rtc_iobuf[3];
		dt.dayOfWeek = rtc_iobuf[4];
		dt.hour      = rtc_iobuf[5];
		dt.minute    = rtc_iobuf[6];
		dt.second    = rtc_iobuf[7];
		rtc_setDateTime (dt);
	    }
	    break;
	case 0x08:  // timer setup
	    timer_inc = v * 10;
	    timer     = millis () + timer_inc;
	    sprintf (sbuf, "\t| timer set to %lu ms", timer_inc);
	    Serial.println (sbuf);
	    break;
	case 0x11:  // disk cmd
	    disk_cmd = v;
	    if (disk_cmd) {
		// read transfer info
		word32 bno = dma_get4 (disk_addr);
		word32 mat = dma_get4 (disk_addr + 4);
		sprintf (sbuf, "\t| disk cmd %c bno %lu mat %04lx", disk_cmd == 2 ? 'W' : 'R', bno, mat); Serial.println (sbuf);

		// make op
		word32 rc = 0;
		if (disk_cmd == 1) {
		    // read
		    if (!sd_readSectors (bno, dma_at (mat), 1)) {
			Serial.println ("\t| IO error disk read block");
			rc = 1;
		    }
		} else {
		    // write
		    if (!sd_writeSectors (bno, dma_at (mat), 1)) {
			Serial.println ("\t| IO error disk write block");
			rc = 2;
		    }
		}

		// ready
		dma_set4 (disk_addr + 8, rc);
		disk_ready = true;
	    }
	    break;
	case 0x12:  // disk addr hi
	    disk_addr = v << 8;
	    break;
	case 0x13:  // disk addr lo
	    disk_addr |= v;
	    switch (disk_cmd) {
		case 0: { // info
			dma_set4 (disk_addr, sd_capacity ());
			disk_ready = true;
		    } break;
		default:
		    sprintf (sbuf, "\t| disk CMD %x ADDR %04x\n", disk_cmd, disk_addr);
		    Serial.print (sbuf);
		    break;
	    }
	    break;
	case 0xff:  // debug notification
	    eon_halt ();
	    break;
	default:
	    sprintf (sbuf, "\t| OUT %lx at %04lx\n", v, port);
	    Serial.print (sbuf);
	    break;
    }
}

// setup: called once to initialize all stuff
void setup () {
    // init console
    Serial.flush ();
    Serial.begin (115200);

    // wire library
    Wire.begin ();

    // outputs
    pinMode (zRUN, OUTPUT);
    pinMode (zRST, OUTPUT);
    pinMode (zBAD, OUTPUT);
    digitalWrite (zRUN, LOW);
    digitalWrite (zRST, LOW);
    digitalWrite (zBAD, LOW);

    // inputs (active low)
    pinMode (zRESET, INPUT_PULLUP);

    // say hello
    Serial.println (F ("\t| eon-esp32 " VERSION));

    // reset cpu
    eon_reset ();

    // RTC setup
    rtc_begin ();

    // sdcard setup
    if (!sd_detect (400000)) {
	sprintf (sbuf, "\t| sdcard init fail with error %i", sd_error ());
	Serial.println (sbuf);
    } else if (!sd_readCSD ()) {
	sprintf (sbuf, "\t| sdcard init fail with error %i", sd_error ());
	Serial.println (sbuf);
    } else {
	sprintf (sbuf, "\t| sdcard present: %lu blocks", (long) sd_capacity ());
	Serial.println (sbuf);
    }

    // timer setup
    timer_inc = 0;
}

// the loop function runs over and over again forever
void loop () {
    // step
    eon_go (64);

    // IRQs
    word32 pending = 0;
    if (Serial.available ()) pending |= IRQ_CON;
    if (disk_ready) {
	pending   |= IRQ_DISK;
	disk_ready = false;
    }
    word32 now;
    if (timer_inc && (now = millis ()) >= timer) {
	timer	 = now + timer_inc;
	pending |= IRQ_TIMER;
	//sprintf (sbuf, "timer ! now %lu next %lu", now, timer); Serial.println (sbuf);
    }
    if (pending) eon_irq (pending);
}
