/*
 * rtc.h
 *
 * RTC device support code, based on DS3221 lib
 * (c) JCGV, junio del 2022
 *
 */
#define DS3231_ADDRESS		    (0x68)
#define DS3231_REG_TIME 	    (0x00)
#define DS3231_REG_CONTROL	    (0x0E)

static uint8_t rtc_readRegister8 (uint8_t reg) {
    uint8_t value;
    Wire.beginTransmission (DS3231_ADDRESS);
    Wire.write (reg);
    Wire.endTransmission();

    Wire.requestFrom (DS3231_ADDRESS, 1);
    while (!Wire.available ())
	;
    value = Wire.read ();
    Wire.endTransmission();

    return value;
}

static void rtc_writeRegister8 (uint8_t reg, uint8_t value) {
    Wire.beginTransmission (DS3231_ADDRESS);
    Wire.write (reg);
    Wire.write (value);
    Wire.endTransmission ();
}

static void rtc_setBattery (bool timeBattery, bool squareBattery) {
    uint8_t value = rtc_readRegister8 (DS3231_REG_CONTROL);
    if (squareBattery)
	value |= 0b01000000;
    else
	value &= 0b10111111;

    if (timeBattery)
	value &= 0b01111011;
    else
	value |= 0b10000000;
    rtc_writeRegister8 (DS3231_REG_CONTROL, value);
}

static int rtc_begin (void) {
    rtc_setBattery (true, false);
    return 0;
}

struct RTCDateTime {
    uint16_t	year;
    uint8_t	month;
    uint8_t	day;
    uint8_t	hour;
    uint8_t	minute;
    uint8_t	second;
    uint8_t	dayOfWeek;
    //uint32_t	  unixtime;
};

static uint8_t bcd2dec (uint8_t bcd) {return ((bcd / 16) * 10) + (bcd % 16);}

static RTCDateTime rtc_getDateTime (void) {
    Wire.beginTransmission (DS3231_ADDRESS);
    Wire.write (DS3231_REG_TIME);
    Wire.endTransmission();

    Wire.requestFrom (DS3231_ADDRESS, 7);
    while (!Wire.available())
	;
    int values[7];
    for (int i = 6; i >= 0; i--)
	values[i] = bcd2dec(Wire.read());
    Wire.endTransmission ();

    RTCDateTime t;
    t.year	= values[0] + 2000;
    t.month	= values[1];
    t.day	= values[2];
    t.dayOfWeek = values[3];
    t.hour	= values[4];
    t.minute	= values[5];
    t.second	= values[6];
    //t.unixtime  = unixtime ();
    return t;
}

static uint8_t dec2bcd (uint8_t dec) {return ((dec / 10) * 16) + (dec % 10);}

static void rtc_setDateTime (RTCDateTime dt) {
    Wire.beginTransmission (DS3231_ADDRESS);
    Wire.write (DS3231_REG_TIME);

    Wire.write (dec2bcd (dt.second));
    Wire.write (dec2bcd (dt.minute));
    Wire.write (dec2bcd (dt.hour));
    Wire.write (dec2bcd (dt.dayOfWeek)); // dow (dt.year, dt.month, dt.day)
    Wire.write (dec2bcd (dt.day));
    Wire.write (dec2bcd (dt.month));
    Wire.write (dec2bcd (dt.year - 2000));

    Wire.write (DS3231_REG_TIME);
    Wire.endTransmission ();
}
