#include <Wire.h>

// This code is simply an implementation of the algorithum
// described on page 12 of the BMP085 data sheet.
// Julian Lewis November 2016

#include <Wire.h>

#define BMP085_ADDRESS 0x77 

const uint8_t OSS = 3;	// High resolution

int16_t  ac1	= 408;
int16_t  ac2	= -72;
int16_t  ac3	= -14383;
uint16_t ac4	= 32741;
uint16_t ac5	= 32757;
uint16_t ac6	= 23153;

int16_t b1	= 6190;
int16_t b2	= 4;

int16_t mb	= -32767;
int16_t mc	= -8711;
int16_t md	= 2868;

uint8_t id	= 0;

int32_t b5; 

// Here is the resulting data from running the algorithum
// by calling BmpGetData

uint16_t raw_temp;
uint32_t raw_pres;

float tru_temp;
float tru_pres;
float tru_alti;

char bmp_deb[128];
char txt[256];

char *BmpDebug() {
	sprintf(bmp_deb,"id:0x%02X ac1:%d ac2:%d ac3:%d ac4:%d ac5:%d ac6:%d b1:%d b2:%d b5:%d mb:%d mc:%d md:%d ut:%d up:%d",
		         id,       ac1,   ac2,   ac3,   ac4,   ac5,   ac6,   b1,   b2,   b5,   mb,   mc,   md,   raw_temp, raw_pres );
	return bmp_deb;
}

uint8_t BmpReadChar(uint8_t address) {
	uint8_t data;

	Wire.beginTransmission(BMP085_ADDRESS);
	Wire.write(address);
	Wire.endTransmission();

	Wire.requestFrom(BMP085_ADDRESS, 1);
	while(!Wire.available()) {};

	return (uint8_t) Wire.read();
}

uint16_t BmpReadShort(uint8_t address) {

	uint8_t msb, lsb;

	Wire.beginTransmission(BMP085_ADDRESS);
	Wire.write(address);
	Wire.endTransmission();

	Wire.requestFrom(BMP085_ADDRESS, 2);
	while(Wire.available() < 2) {};

	msb = Wire.read();
	lsb = Wire.read();

	return (uint16_t) (msb << 8 | lsb);
}

void BmpReadCalibData() {

	static int calib_ok = 0;
	if (calib_ok) return;
	
	id  = BmpReadChar(0xD0);
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

uint16_t BmpReadRawTemp() {
	uint16_t ut;

	Wire.beginTransmission(BMP085_ADDRESS);
	Wire.write(0xF4);
	Wire.write(0x2E);
	Wire.endTransmission();

	delay(5);

	ut = BmpReadShort(0xF6);
	return ut;
}

uint32_t BmpReadRawPres() {
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

int32_t BmpCalcB5(int32_t ut) {
	int32_t x1;
	int32_t x2;

	x1 = (ut - (int32_t) ac6) * ((int32_t) ac5) >> 15;
	x2 = ((int32_t) mc << 11) / (x1 + (int32_t) md);
	return x1 + x2;
}

float BmpCalcTemp() {

	BmpReadCalibData();
	raw_temp = BmpReadRawTemp();
	b5 = BmpCalcB5(raw_temp);
	return (float) ((b5 + 8) >> 4) / 10.0;
}

float BmpCalcPres() {

	int32_t x1, x2, x3, b3, b6, p;
	uint32_t b4, b7;

	BmpReadCalibData();
	raw_temp = BmpReadRawTemp();
	raw_pres = BmpReadRawPres();
	b5 = BmpCalcB5(raw_temp);

	b6 = b5 - 4000;

	x1 = (b2 * ((b6 * b6) >> 12)) >> 11;
	x2 = (ac2 * b6)>>11;
	x3 = x1 + x2;
	b3 = (((((int32_t) ac1) * 4 + x3) << OSS) + 2) >> 2;

	x1 = (ac3 * b6) >> 13;
	x2 = (b1 * ((b6 * b6) >> 12)) >> 16;
	x3 = ((x1 + x2) + 2) >> 2;
	b4 = (ac4 * (uint32_t) (x3 + 32768)) >> 15;

	b7 = ((uint32_t) (raw_pres - b3) * (50000 >> OSS));
	if (b7 < 0x80000000)
		p = (b7 << 1) / b4;
	else
		p = (b7 / b4) << 1;

	x1 = (p >> 8) * (p >> 8);
	x1 = (x1 * 3038) >> 16;
	x2 = (-7357 * p) >> 16;

	return (float) (p + ((x1 + x2 + 3791) >> 4)) / 100.0;
}

float BmpCalcAlti() {

	float pp0, pres;
	
	pres = BmpCalcPres();
	pp0 = pres/1013.25;
	pp0 = pow(pp0,(1.0/5.255));
	pp0 = 1 - pp0;
	return 44330.0 * pp0;
}

void BmpGetData() {

	tru_temp = BmpCalcTemp();
	tru_pres = BmpCalcPres();
	tru_alti = BmpCalcAlti();
}

#if 0
void setup() {
	Serial.begin(9600);
	Wire.begin();	
}

void loop() {
	BmpGetData();
	BmpDebug();
	Serial.println(bmp_deb);
	sprintf(txt,"temp:%3.2f pres:%3.2f alti:%3.2f\n",tru_temp,tru_pres,tru_alti);
	Serial.println(txt);
	delay(5000);
}
#endif
