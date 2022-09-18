# eon-esp32
[EON](https://github.com/elgron-eon/eon-cpu) cpu system with ESP8266/ESP32. An emulator is included.
This is a very simple 32 bit EON cpu system, with 8KB ROM and 32/128KB RAM.
The estimated clock frecuency is about 8MHz.

# hardware parts
* esp8266 or esp32
* DS1307 serial RTC, I2C, 3.3 or 5v
* breadboard, one resistor and dupont wires

# build
Prerequisites:
* eonrom.img from [eonrom](https://github.com/elgron-eon/eonrom)
* [arduino-cli](https://github.com/arduino/arduino-cli)
* a serial terminal emulator. I use [picocom](https://github.com/npat-efault/picocom), but any other will work.

to build emulator, just type `make`  
to build arduino image, type `make compile`  
and finally type `make upload && make com` to enjoy your system !

# diagram
check the gallery images. From top to bottom:
* RTC clock
* SPI card reader

# gallery
![foto1](photo/foto1.jpg?raw=true)

# esp8266 pinout
* reset button: pin13(active low)
* leds: pin10(RUN) pin11(RESET) pin12(ERROR)
* i2c bus: pin20(SDA) pin21(SCL)
* spi bus: pin52(SCK) pin50(MISO) pin 51(MOSI) ping 53(SS active low)

# rtc pinout
```
    unconnected -+
    unconnected -+
            SCL -+
            SDA -+
            VCC -+
            GND -+
```

