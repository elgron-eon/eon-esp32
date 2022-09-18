ifdef ESP32
BOARD = esp32:esp32:nodemcu-32s
PORT  = /dev/ttyUSB0
BFLAG = BOARD_ESP32
else
BOARD = esp8266:esp8266:nodemcuv2
PORT  = /dev/ttyUSB0
BFLAG = BOARD_ESP8266
endif

PROJECT = main
SRCS	= cpu.h dcache.h rom.h rtc.h sdcard.h

all: zeon

clean:
	rm -fr ${PROJECT} zeon rom.h

/tmp/sdcard.disk: sdcard.gpt
	dd if=/dev/zero of=$@ bs=1M count=512 status=none
	gpttool -mbr:1 -v $@ sdcard.gpt

rom.h: eonrom.bin
	@echo "generating rom.h ..."
	@echo "static const unsigned char zROM[8192] PROGMEM = {" > $@
	@bin2c $^  >> $@
	@echo "};" >> $@

zeon: emulator.c ${PROJECT}.ino ${SRCS} /tmp/sdcard.disk
	musl-gcc -D_GNU_SOURCE -s -std=c11 -O2 -Wall -o $@ emulator.c

compile: ${PROJECT}.ino ${SRCS}
	rm -fr ${PROJECT} && mkdir ${PROJECT} && cp $^ ${PROJECT}/
	arduino-cli compile -e -b ${BOARD} --build-property "build.extra_flags=-D${BFLAG}" ${PROJECT}

upload:
	arduino-cli upload -b ${BOARD} ${PROJECT} -p ${PORT}

com:
	picocom -b 115200 -r -l -f n --imap lfcrlf ${PORT}

