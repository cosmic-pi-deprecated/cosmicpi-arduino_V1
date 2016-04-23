/*************************************************** 
  This is a library for the HTU21DF Humidity & Temp Sensor

  Designed specifically to work with the HTU21DF sensor from Adafruit
  ----> https://www.adafruit.com/products/1899

  These displays use I2C to communicate, 2 pins are required to  
  interface
  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include "Adafruit_HTU21DF.h"
//#include <util/delay.h>

Adafruit_HTU21DF::Adafruit_HTU21DF() {
}


boolean Adafruit_HTU21DF::begin(void) {
  Wire1.begin();
  
  reset();

  Wire1.beginTransmission(HTU21DF_I2CADDR);
  Wire1.write(HTU21DF_READREG);
  Wire1.endTransmission();
  Wire1.requestFrom(HTU21DF_I2CADDR, 1);
  return (Wire1.read() == 0x2); // after reset should be 0x2
}

void Adafruit_HTU21DF::reset(void) {
  Wire1.beginTransmission(HTU21DF_I2CADDR);
  Wire1.write(HTU21DF_RESET);
  Wire1.endTransmission();
  delay(15);
}


float Adafruit_HTU21DF::readTemperature(void) {
  
  // OK lets ready!
  Wire1.beginTransmission(HTU21DF_I2CADDR);
  Wire1.write(HTU21DF_READTEMP);
  Wire1.endTransmission();
  
  delay(50); // add delay between request and actual read!
  
  Wire1.requestFrom(HTU21DF_I2CADDR, 3);
  while (!Wire1.available()) {}

  uint16_t t = Wire1.read();
  t <<= 8;
  t |= Wire1.read();

  uint8_t crc = Wire1.read();

  float temp = t;
  temp *= 175.72;
  temp /= 65536;
  temp -= 46.85;

  return temp;
}
  

float Adafruit_HTU21DF::readHumidity(void) {
  // OK lets ready!
  Wire1.beginTransmission(HTU21DF_I2CADDR);
  Wire1.write(HTU21DF_READHUM);
  Wire1.endTransmission();
  
  delay(50); // add delay between request and actual read!
  
  Wire1.requestFrom(HTU21DF_I2CADDR, 3);
  while (!Wire1.available()) {}

  uint16_t h = Wire1.read();
  h <<= 8;
  h |= Wire1.read();

  uint8_t crc = Wire1.read();

  float hum = h;
  hum *= 125;
  hum /= 65536;
  hum -= 6;

  return hum;
}



/*********************************************************************/
