// This code is simply an implementation of the algorithum
// described on page 12 of the BMP085 data sheet.
// Julian Lewis November 2016

#include <Wire.h>

#define BMP085_ADDRESS 0x77 

const uint8_t OSS = 1;	// Normal 

static int16_t  ac1	= 408;
static int16_t  ac2	= -72;
static int16_t  ac3	= -14383;
static uint16_t ac4	= 32741;
static uint16_t ac5	= 32757;
static uint16_t ac6	= 23153;

static int16_t b1	= 6190;
static int16_t b2	= 4;

static int16_t mb	= -32767;
static int16_t mc	= -8711;
static int16_t md	= 2868;

static int32_t b5; 

static int16_t	BmpReadShort(uint8_t address);
static void	BmpReadCalibData();
static int32_t	BmpCalcTemp(uint16_t ut);
static int32_t	BmpCalcPres(uint32_t up);
static int8_t	BmpReadChar(uint8_t address);
static uint16_t	BmpReadRawTemp();
static uint32_t	BmpReadRawPres();
static float	BmpCalcAlti(int32_t pres);

// Here is the resulting data from running the algorithum
// by calling BmpGetData

uint16_t	raw_temp;
uint32_t	raw_pres;
int32_t		tru_temp;
int32_t		tru_pres;
float		tru_alti;
char		bmp_deb[128];

void BmpGetData() {

	BmpReadCalibData();
	raw_temp = BmpReadRawTemp();
	raw_pres = BmpReadRawPres();
	tru_temp = BmpCalcTemp(raw_temp);
	tru_pres = BmpCalcPres(raw_pres);
	tru_alti = BmpCalcAlti(tru_pres);
}

char *BmpDebug() {
	sprintf(bmp_deb,"ac1:%d ac2:%d ac3:%d ac4:%d ac5:%d ac6:%d b1:%d b2:%d mb:%d mc:%d md:%d, ut:%d up:%d",
		         ac1,   ac2,   ac3,   ac4,   ac5,   ac6,   b1,   b2,   mb,   mc,   md, raw_temp, raw_pres );
	return bmp_deb;
}

static int16_t BmpReadShort(uint8_t address) {

	uint8_t msb, lsb;

	Wire.beginTransmission(BMP085_ADDRESS);
	Wire.write(address);
	Wire.endTransmission();

	Wire.requestFrom(BMP085_ADDRESS, 2);
	while(Wire.available() < 2) {};

	msb = Wire.read();
	lsb = Wire.read();

	return (int16_t) (msb << 8 | lsb);
}

static void BmpReadCalibData() {

	static int calib_ok = 0;
	if (calib_ok) return;

	ac1 = BmpReadShort(0xAA);
	ac2 = BmpReadShort(0xAC);
	ac3 = BmpReadShort(0xAE);
	ac4 = BmpReadShort(0xB0);
	ac5 = BmpReadShort(0xB2);
	ac6 = BmpReadShort(0xB4);
	b1  = BmpReadShort(0xB6);
	b2  = BmpReadShort(0xB8);
	mb  = BmpReadShort(0xBA);
	mc  = BmpReadShort(0xBC);
	md  = BmpReadShort(0xBE);
	BmpDebug();
	calib_ok = 1;
}

static int32_t BmpCalcTemp(uint16_t ut) {

	int32_t x1, x2;

	x1 = (int32_t) ((ut - (uint32_t) ac6) * (uint32_t) ac5) >> 15;
	x2 = (int32_t) (((uint32_t) mc << 11) / (x1 + (uint32_t) md));
	b5 = x1 + x2;
	return ((b5 + 8) >> 4);
}

static int32_t BmpCalcPres(uint32_t up) {

	int32_t x1, x2, x3, b3, b6, p;
	uint32_t b4, b7;

	b6 = b5 - 4000;

	x1 = (b2 * ((b6 * b6) >> 12)) >> 11;
	x2 = (ac2 * b6)>>11;
	x3 = x1 + x2;
	b3 = ((((uint32_t) ac1 * 4) + x3) << (OSS + 2)) >> 2;

	x1 = (ac3 * b6) >> 13;
	x2 = (b1 * ((b6 * b6) >> 12)) >> 16;
	x3 = ((x1 + x2) + 2) >> 2;
	b4 = (ac4 * (uint32_t) (x3 + 32768)) >> 15;

	b7 = ((uint32_t) (up - b3) * (50000 >> OSS));
	if (b7 < 0x80000000)
		p = (b7 << 1) / b4;
	else
		p = (b7 / b4) << 1;

	x1 = (p >> 8) * (p >> 8);
	x1 = (x1 * 3038) >> 16;
	x2 = (-7357 * p) >> 16;
	p += (x1 + x2 + 3791) >> 4;

	return p;
}

static int8_t BmpReadChar(uint8_t address) {
	uint8_t data;

	Wire.beginTransmission(BMP085_ADDRESS);
	Wire.write(address);
	Wire.endTransmission();

	Wire.requestFrom(BMP085_ADDRESS, 1);
	while(!Wire.available()) {};

	return Wire.read();
}

static uint16_t BmpReadRawTemp() {
	uint16_t ut;

	Wire.beginTransmission(BMP085_ADDRESS);
	Wire.write(0xF4);
	Wire.write(0x2E);
	Wire.endTransmission();

	delay(5);

	ut = BmpReadShort(0xF6);
	return ut;
}

static uint32_t BmpReadRawPres() {
	uint8_t msb, lsb, xlsb;
	uint32_t up = 0;

	Wire.beginTransmission(BMP085_ADDRESS);
	Wire.write(0xF4);
	Wire.write(0x34 + (OSS<<6));
	Wire.endTransmission();

	delay(2 + (3 << OSS));

	msb = BmpReadChar(0xF6);
	lsb = BmpReadChar(0xF7);
	xlsb = BmpReadChar(0xF8);

	up = (uint32_t) (( msb << 16) | (lsb << 8) | xlsb) >> (8-OSS);

	return up;
}

static float BmpCalcAlti(int32_t pres) {

	float pp0;

	pp0 = pres/1013.25;
	pp0 = pow(pp0,(1.0/5.255));
	pp0 = 1 - pp0;
	return 44330.0 * pp0;
}
