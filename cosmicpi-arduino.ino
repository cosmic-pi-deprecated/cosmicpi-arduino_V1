//initialisation code
#include <time.h>
#include <Wire.h>
#include <math.h>

#include "Adafruit_BMP085_U.h"  // Barrometric pressure
#include "Adafruit_HTU21DF.h" // Humidity and temperature sensor
#include "Adafruit_GPS.h" // GPS chip
#include "Adafruit_L3GD20_U.h"  // Magoscope
// WARNING: I had to modify this library, its no longer standard 
#include "Adafruit_LSM303_U.h"  // Accelerometer and magnentometer/compass
#include "Adafruit_10DOF.h" // 10DOF breakout driver - scale to SI units



//setup variables
#define VERS "2016/Apr/24"
#define ADC_BUF_LEN 300  // Number of ADC values per event
#define PPS_EVENTS 10  // The maximum number of events stored per second
#define EVENT_QSIZE 32  // The number of events that can be queued for serial output
#define ACL_PIN 10  // Accelarometer INT1 interrupt pin
#define GPS_BAUD_RATE 9600  // GPS and Serial1 line
#define BMPID 18001 //barometric pressure sensor
#define ACLID 30301 //accelerometer definition
#define MAGID 30302 //magnetometer definition
#define SERIAL_BAUD_RATE 9600 // Serial line 
int textctr = 0;



//setup hardware breakouts

Adafruit_GPS    gps(&Serial1);      // GPS Serial1 on pins RX1 and TX1

Adafruit_HTU21DF  htu = Adafruit_HTU21DF(); // Humidity and temperature measurment
boolean     htu_ok = false;     // Chip OK

Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(BMPID); // Barometric pressure
boolean     bmp_ok = false;

// The 10DOF isn't a chip, its just a utility to convert say mago values into headings etc
Adafruit_10DOF    dof = Adafruit_10DOF();   // The 10 Degrees-Of-Freedom DOF breakout
boolean     dof_ok = false;     // board driver, scales units to SI
Adafruit_LSM303_Accel_Unified acl = Adafruit_LSM303_Accel_Unified(ACLID); // Accelerometer Compass
      boolean acl_ok = false;

Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(MAGID);   // Magoscope
boolean     mag_ok = false;

//init measurement values and rates
uint32_t latlon_display_rate = 12;  // Display latitude and longitude each X seconds
uint32_t humtmp_display_rate = 12;  // Display humidity and HTU temperature each X seconds
uint32_t alttmp_display_rate = 12;  // Display altitude and BMP temperature each X seconds
uint32_t frqutc_display_rate = 1; // Display frequency and UTC time each X seconds
uint32_t status_display_rate = 4; // Display status (UpTime, QueueSize, MissedEvents, HardwareOK)
uint32_t accelr_display_rate = 1; // Display accelarometer x,y,z
uint32_t magnot_display_rate = 12;  // Display magnotometer data x,y,z
uint32_t events_display_size = 20;  // Display events after recieving X events

// Siesmic event trigger parameters
uint32_t accelr_event_threshold = 2;  // Trigger level for siesmic events in milli-g
uint32_t accelr_event_cutoff_fr = 30; // Siesmic event cutoff frequency


//define serial output ringbuffer
#define TBLEN 4096  // Serial line output ring buffer size
static char txtb[TBLEN];    // Text ring buffer
static uint32_t txtw = 0, txtr = 0,   // Write and Read indexes
    tsze = 0, terr = 0; // Buffer size and error code
typedef enum { TXT_NOERR=0, TXT_TOOBIG=1, TXT_OVERFL=2 } TxtErr;

//serial output buffers
#define TXTLEN 256
static byte txt[TXTLEN];    // For writing to serial  

//timer initialisation
//PPS input to D2
//Trigger input to D5
//Clock rate is 83MHz/2, so approx 41.5Mhz

void TimersStart() {
uint32_t config = 0;

// Set up the power management controller for TC0 and TC2
pmc_set_writeprotect(false);    // Enable write access to power management chip
pmc_enable_periph_clk(ID_TC0);  // Turn on power for timer block 0 channel 0
pmc_enable_periph_clk(ID_TC6);  // Turn on power for timer block 2 channel 0

  // Timer block zero channel zero is connected only to the PPS 
  // We set it up to load register RA on each PPS and reset
  // So RA will contain the number of clock ticks between two PPS, this
  // value should be very stable +/- one tick

config = TC_CMR_TCCLKS_TIMER_CLOCK1 |        // Select fast clock MCK/2 = 42 MHz
         TC_CMR_ETRGEDG_RISING |             // External trigger rising edge on TIOA0
         TC_CMR_ABETRG |                     // Use the TIOA external input line
         TC_CMR_LDRA_RISING;                 // Latch counter value into RA

TC_Configure(TC0, 0, config);                // Configure channel 0 of TC0
TC_Start(TC0, 0);                            // Start timer running

TC0->TC_CHANNEL[0].TC_IER =  TC_IER_LDRAS;   // Enable the load AR channel 0 interrupt each PPS
TC0->TC_CHANNEL[0].TC_IDR = ~TC_IER_LDRAS;   // and disable the rest of the interrupt sources
NVIC_EnableIRQ(TC0_IRQn);                    // Enable interrupt handler for channel 0


// Timer block 2 channel zero is connected to the Trigger signal from a cosmic ray event
 
 config = TC_CMR_TCCLKS_TIMER_CLOCK1 |        // Select fast clock MCK/2 = 42 MHz
          TC_CMR_ETRGEDG_RISING |             // External trigger rising edge on TIOA1
          TC_CMR_ABETRG |                     // Use the TIOA external input line
          TC_CMR_LDRA_RISING;                 // Latch counter value into RA
  
TC_Configure(TC2, 0, config);                // Configure channel 0 of TC2
TC_Start(TC2, 0);          // Start timer running
 
TC2->TC_CHANNEL[0].TC_IER =  TC_IER_LDRAS;   // Enable the load AR channel 0 interrupt each PPS
TC2->TC_CHANNEL[0].TC_IDR = ~TC_IER_LDRAS;   // and disable the rest of the interrupt sources
NVIC_EnableIRQ(TC6_IRQn);                    // Enable interrupt handler for channel 0

// Set up the PIO controller to route input pins for TC0 and TC2
PIO_Configure(PIOC,PIO_INPUT,
          PIO_PB25B_TIOA0,  // D2 Input 
          PIO_DEFAULT);
PIO_Configure(PIOC,PIO_INPUT,
          PIO_PC25B_TIOA6,  // D5 Input
          PIO_DEFAULT);
}

// Timer0 system interrupt handlers and variables
static uint32_t ppsfl = LOW,  // PPS Flag boolean
                rega0 = 0,  // RA reg
                stsr0 = 0,  // Interrupt status register
                ppcnt = 0;  // PPS count

// Handle the PPS interrupt in counter block 0 ISR

void TC0_Handler() {
// This ISR is run only when the PPS (Pulse Per Second) GPS event is detected
rega0 = TC0->TC_CHANNEL[0].TC_RA; // Read the RA reg (PPS period) and store in rega0
stsr0 = TC_GetStatus(TC0, 0);     // Read status and clear load bits, reset interrupt on exit
ppcnt++;        // PPS count increase
}

// Timer0 system interrupt handlers and variables
static uint32_t rega1 = 0, //register for TC6 to be written into
                eventcnt = 0, //event counter
                stsr1 = 0; //Interrupt status register

void TC6_Handler() {
// This ISR is connected to the event trigger 

//insert here the indexing of the timer for events and ADC reading
//dump the value of the timer at event into the event tcks variable
//wbuf[widx].Tks = TC0->TC_CHANNEL[0].TC_CV;
//get the ADC data, copy the DMA buffer into a new buffer
//AdcPullData(&wbuf[widx]);
//      widx++;
  eventcnt++;
  stsr1 = TC_GetStatus(TC2, 0);     // Read status clear load bits, reset interrupt
}


//Accelerometer setup
void AclSetup() {
uint8_t tmp, val;
//abort if accelerometer status if fault.
if (!acl_ok) return;

//define accelerometer setuo variables
#define PMD 0x20  // Normal power mode (PM0=1,PM1=0:Normal)
#define DRT 0x00  // Data rate 50 Hz (0x08 = 100Hz)
#define AEN 0x07  // XYZ Enabled

//concatenate and write to accelerometer chip
val = PMD | DRT | AEN;
acl.write8(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_CTRL_REG1_A, val);

//setup frequency filters for acceleration events
#define HPE1 0x04 // High pass filter Int 1 on
#define HPCF 0x03 // High pass cut off frequency

//concatenate and write to accelerometer chip
val = HPE1 | HPCF;
acl.write8(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_CTRL_REG2_A, val);

//Define latching behaviour
#define LIR1 0x06 // Latch Int1 bit Data ready
#define LIR2 0x00 // Latch Int2 bit Data ready (0x20 Latch On)
#define IHL_OD 0xC0 // Interrupt active low, open drain (Argh !!!)

//concatenate and write to accelerometer chip
val = LIR1 | LIR2 | IHL_OD;
acl.write8(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_CTRL_REG3_A, val);

//define internal gain of accelerometer
#define BDU_FS 0x80 // Block data and scale +-2g

//concatenate and write to accelerometer chip
val = BDU_FS;
acl.write8(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_CTRL_REG4_A, val);

//define directions and maximum values
#define XYZ_HI 0x2A // Hi values ZHIE YHIE XHIE
#define AOI_6D 0x00 // 0xC0 would enable 6 directions

//concatenate and write to accelerometer chip
val = XYZ_HI | AOI_6D;
acl.write8(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_INT1_CFG_A, val);

//set threshold
val = accelr_event_threshold & 0x7F;
acl.write8(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_INT1_THS_A, val);

//attach interrupt for acceleration events  
attachInterrupt(digitalPinToInterrupt(ACL_PIN),Acl_ISR,RISING);
//accelerometer setup complete
}

// Magnatometer setup, again the Adda_fruit library was inadequate. 

void MagSetup() {
uint8_t val;
//abort if magnetometer status fault
if (!mag_ok) return;

//initialise variables on mag board
val = 0;
mag.write8(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_CRA_REG_M, val);

//set the gain
#define GAIN 0x80 // +- 4.0 Gauss
val = GAIN;
mag.write8(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_CRB_REG_M, val);

//define conversion mode
#define MODE 0x0  // 01=Single conversion mode
val = MODE;
mag.write8(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_MR_REG_M, val);
//magnetometer setup completed
}

//Accelerometer ISR
static uint32_t accl_icount = 0, accl_flag = 0;
void Acl_ISR() {
//Set the bitmask for accelerometer interrupt
#define IA 0x40
//increment accelerometer event count
accl_icount++;
//read the device status into variable
accl_flag = AclReadStatus();
//launch transmission routine for this variable
//PushVib();
}

// Read accelerometer status
// This just reads the interrupt source INT1 and the overrun status.
// It returns 1 bit for X, Y, or Z (0..7) if the threshold value is exceeded.
// This determins if the board is being shaken - Earth quake - or other reason
static uint8_t acl_sts = 0;
static uint8_t acl_src = 0;
uint8_t AclReadStatus() {
  uint8_t rval;
  acl_src = acl.read8(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_INT1_SOURCE_A);

#define ZH 0x20 // Z High
#define YH 0x08 // Y High
#define XH 0x02 // X High

  rval = 0;
  if (acl_src & IA) {
    if (acl_src & ZH) rval |= 4;
    if (acl_src & YH) rval |= 2;
    if (acl_src & XH) rval |= 1;
  }
  if (rval) 
    acl_sts = acl.read8(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_STATUS_REG_A);
  return rval;
}


// Set up the ADC channels
void AdcSetup() {
  ADC->ADC_MR = 0x10380180; // Free running maximum speed
  ADC -> ADC_CHER = 0x03; // enable ADC on pin A6 and A7
}

// Extract NMEA data string from the GPS chip

//define required variables
#define GPS_STRING_LEN 256
static char gps_string[GPS_STRING_LEN + 1];
float latitude = 0.0, longitude = 0.0, altitude = 0.0;
// We also need a time value for the current and previous second
#ifdef RMCGGA 
#define DATE_TIME_LEN 9
#else
#define DATE_TIME_LEN 17
#endif

static char t1[DATE_TIME_LEN];    // Date time buffer text string
static char t2[DATE_TIME_LEN];    
static char *wdtm = t1;     // Write date/time pointer
static char *rdtm = t2;     // Read date/time pointer

// This function is dependent on the GPS chip implementation
// It should return a date time string as described above
// So you need to re-implements this for whichever chip you are using
// Here I am using the addafruit GPS chip

char *GetDateTime() {
  int i = 0;
  while (Serial1.available()) {
    if (i < GPS_STRING_LEN) {
      gps_string[i++] = (char) Serial1.read();
      gps_string[i] = 0;
    } else i++;
  }
  if (gps.parse(gps_string)) {

#ifdef RMCGGA 
    // I choose RMCGGA by default, and get the altitude but no date.
    // Its easy to get the date once the records arrive at the Python end.
    // The GPS altitude is far more accurate than the barrometric altitude.
    // Warning: The syntax can not be changed, we need an integer hhmmss
sprintf(wdtm,
        "%02d%02d%02d",
        gps.hour,
        gps.minute,
        gps.seconds);
altitude  = gps.altitude; 
#else
sprintf(wdtm,
        "%02d%02d%02d%02d%02d%02d%02d%02d",
        gps.year,
        gps.month,
        gps.day,
        gps.hour,
        gps.minute,
        gps.seconds);

altitude = 0;
#endif      
    latitude  = gps.latitudeDegrees;  // Easy place to get location
    longitude = gps.longitudeDegrees; // Works well in Google maps
    return rdtm;
  } else
    return NULL;
}


//Create queue system
// Implement queue access mechanism for events, each second the user space (loop) copies
// any events it has read onto the queue
struct EventBuf {
  char    DateTime[DATE_TIME_LEN];  // The date and time string
  uint32_t  Frequency;      // The current clock frequency
  uint32_t  Ticks;        // The number of ticks since the last event or PPS if none
  uint16_t  Ch0[ADC_BUF_LEN];   // ADC channel 0 values
  uint16_t  Ch1[ADC_BUF_LEN];   // ADC channel 1 values
  uint8_t   Count;        // The number of events since the PPS
};

typedef struct {
  uint8_t   Size;       // Current size of the queue
  uint8_t   RdPtr;        // Read pointer
  uint8_t   WrPtr;        // Write pointer
  uint8_t   Missed;       // Missed events counter due to overflow
  uint8_t   Lock;       // The queue spin lock (not needed here)
  struct EventBuf Events[EVENT_QSIZE];    // Queued events 
} EventQueue;

static EventQueue event_queue;

// Put an event in an EventBuf on the queue, if the queue is full then the oldest event
// is thrown away and the "missed" event count is incremented

uint8_t PutQueue(struct EventBuf *ebuf) {

  EventQueue *q = &event_queue;

  while(q->Lock) {}; q->Lock = 1;   // Spin lock on queue
  q->Events[q->WrPtr] = *ebuf;    // Write event to the queue
  q->WrPtr = (q->WrPtr + 1) % EVENT_QSIZE;// Increment the write pointer
  if (q->Size < EVENT_QSIZE) q->Size++; // If we are overwriting old enties that havnt been read
  else {
    q->Missed++;          // Say we missed some events
    q->RdPtr = (q->RdPtr + 1) % EVENT_QSIZE;  // and throw the oldest event away  
  }
  q->Lock = 0;
  return q->Missed;
}

// Pop an event off the queue, if the queue is empty nothing happens
// the queue size is zero when the queue is empty, and this is the
// return value

uint8_t PopQueue(struct EventBuf *ebuf) { // Points to where the caller wants the event stored

  EventQueue *q = &event_queue;

  while(q->Lock) {}; q->Lock = 1;   // Spin lock on queue
  if (q->Size) {
    *ebuf = q->Events[q->RdPtr];
    q->RdPtr = (q->RdPtr + 1) % EVENT_QSIZE;
    q->Size--;
  }
  q->Lock = 0;
  return q->Size; 
}

// Get the size of the queue

uint8_t SzeQueue() {

  EventQueue *q = &event_queue;

  return q->Size;
}
 
// Initialize the queue

void InitQueue() {

  EventQueue *q = &event_queue;

  q->Lock = 1;
  q->Size = 0;
  q->RdPtr = 0;
  q->WrPtr = 0;
  q->Missed = 0;
  q->Lock = 0;
}



void setup() {
Serial.begin(9600);
SerialUSB.begin(0); // Start the serial line
Serial1.begin(GPS_BAUD_RATE); // and the second

gps.begin(GPS_BAUD_RATE); // Chip baud rate

#ifdef RMCGGA
  gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // With altitude but no yy/mm/dd
#else
  gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);  // With yy/mm/dd but no altitude
#endif
  gps.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);  // each second


//InitQueue();      // Reset queue pointers, missed count, and size

strcpy(rdtm,"");    // Set initial value for date/time
strcpy(wdtm,"");

htu_ok = htu.begin();
bmp_ok = bmp.begin();
acl_ok = acl.begin();
mag_ok = mag.begin();
dof_ok = dof.begin();

AclSetup();
MagSetup();
//AdcSetup();
  
TimersStart();      // Start timers
}

void PushHtu(int flg) { // If flg is true always push

  float temph = 0.0; 
  float humid = 0.0;
//Serial.println("testingHTU");
  if ((flg) || ((htu_ok) && ((ppcnt % humtmp_display_rate) == 0))) {
    //Serial.println("reading");
    temph = htu.readTemperature();
    humid = htu.readHumidity();
    //sprintf(txt,"{'HTU':{'Tmh':%5.3f,'Hum':%4.1f}}\n",temph,humid);
    txt[textctr] = 1;//code for htu
    textctr++;
    byte *bufbyteone = (byte *)&temph;
    memcpy(&txt[textctr],&bufbyteone[0],sizeof(bufbyteone));
    textctr = textctr + sizeof(bufbyteone);
    byte *bufbytetwo = (byte *)&humid;
    memcpy(&txt[textctr],&bufbytetwo[0],sizeof(bufbytetwo));
    textctr = textctr + sizeof(bufbytetwo);
    txt[textctr] = 10;
    textctr++;
    Serial.println(1);
    Serial.println(temph);
    Serial.println(humid);
    Serial.println();
  }
}

// Push BMP temperature and altitude from BMP chip

void PushBmp(int flg) { // If flg is true always push

  float  altib = 0.0;
  float  tempb = 0.0;
  float  presr = 0.0;
  sensors_event_t bmp_event;  // Barrometric pressure event   

  if ((flg) || ((bmp_ok) && ((ppcnt % alttmp_display_rate) == 0))) {
    bmp.getEvent(&bmp_event);
    if (bmp_event.pressure) {
      presr = bmp_event.pressure;
      bmp.getTemperature(&tempb);
      altib = bmp.pressureToAltitude((float) SENSORS_PRESSURE_SEALEVELHPA, 
              presr,tempb);
      //sprintf(txt,"{'BMP':{'Tmb':%5.3f,'Prs':%5.3f,'Alb':%4.1f}}\n",tempb,presr,altib);
      //PushTxt(txt);
    txt[textctr] = 2;//code for htu
    textctr++;
    byte *bufbytethree = (byte *)&tempb;
    memcpy(&txt[textctr],&bufbytethree[0],sizeof(bufbytethree));
    textctr = textctr + sizeof(bufbytethree);
    byte *bufbytefour = (byte *)&presr;
    memcpy(&txt[textctr],&bufbytefour[0],sizeof(bufbytefour));
    textctr = textctr + sizeof(bufbytefour);
    byte *bufbytefive = (byte *)&altib;
    memcpy(&txt[textctr],&bufbytefive[0],sizeof(bufbytefive));
    textctr = textctr + sizeof(bufbytefive);
    txt[textctr] = 10;
    textctr++;      
    Serial.println(2);
    Serial.println(tempb);
    Serial.println(presr);
    Serial.println(altib);
    Serial.println(textctr);
    Serial.println();
    Serial.println(txt[0],DEC);
    Serial.println(txt[1],DEC);
    Serial.println(txt[2],DEC);
    Serial.println(txt[3],DEC);
    Serial.println(txt[3],DEC);
    Serial.println(txt[4],DEC);
    Serial.println(txt[5],DEC);
    Serial.println(txt[6],DEC);
    Serial.println(txt[7],DEC);
    Serial.println(txt[8],DEC);
    Serial.println(txt[9],DEC);
    Serial.println(txt[10],DEC);
    Serial.println(txt[11],DEC);
    Serial.println(txt[12],DEC);
    Serial.println(txt[13],DEC);
    Serial.println("endofbytes");
    }
  }
}

void PushLoc(int flg) {
    
  if ((flg) || ((ppcnt % latlon_display_rate) == 0)) {
    //sprintf(txt,"{'LOC':{'Lat':%f,'Lon':%f,'Alt':%f}}\n",latitude,longitude,altitude);
    //PushTxt(txt);
    txt[textctr] = 8;//code for htu
    textctr++;
    byte *bufbytesix = (byte *)&latitude;
    memcpy(&txt[textctr],&bufbytesix[0],sizeof(bufbytesix));
    textctr = textctr + sizeof(bufbytesix);
    byte *bufbyteseven = (byte *)&longitude;
    memcpy(&txt[textctr],&bufbyteseven[0],sizeof(bufbyteseven));
    textctr = textctr + sizeof(bufbyteseven);
    byte *bufbyteeight = (byte *)&altitude;
    memcpy(&txt[textctr],&bufbyteeight[0],sizeof(bufbyteeight));
    textctr = textctr + sizeof(bufbyteeight);
    txt[textctr] = 10;
    textctr++;      
    
    Serial.println(8);
    Serial.println(latitude);
    Serial.println(longitude);
    Serial.println(altitude);
    Serial.println();
  }
}

void PushTim(int flg) {
//time not really working - no pps for testing
  if ((flg) || ((ppcnt % frqutc_display_rate) == 0)) {
    //sprintf(txt,"{'TIM':{'Upt':%4d,'Frq':%7d,'Sec':%s}}\n",ppcnt,rega0,rdtm);
    //PushTxt(txt);
    txt[textctr] = 9;//code for htu
    textctr++;
    byte *bufbytenine = (byte *)&ppcnt;
    memcpy(&txt[textctr],&bufbytenine[0],sizeof(bufbytenine));
    textctr = textctr + sizeof(bufbytenine);
    byte *bufbyteten = (byte *)&rega0;
    memcpy(&txt[textctr],&bufbyteten[0],sizeof(bufbyteten));
    textctr = textctr + sizeof(bufbyteten);
    byte *bufbyteeleven = (byte *)&rdtm;
    memcpy(&txt[textctr],&bufbyteeleven[0],sizeof(bufbyteeleven));
    textctr = textctr + sizeof(bufbyteeleven);
    txt[textctr] = 10;
    textctr++;  
    Serial.println(9);
    Serial.println(ppcnt);
    Serial.println(rega0);
    Serial.println(rdtm);
    Serial.println();
  }     
}

void PushMag(int flg) { // Push the mago stuff
  sensors_event_t mag_event;
  sensors_vec_t xyz;
  Serial.print("magsensstart");
  //if ((flg) || ((mag_ok) && ((ppcnt % magnot_display_rate) == 0))) {
    mag.getEvent(&mag_event);
    Serial.print("magread");
    // Micro Tesla

    //sprintf(txt,"{'MAG':{'Mgx':%f,'Mgy':%f,'Mgz':%f}}\n",
    //  mag_event.magnetic.x,
    //  mag_event.magnetic.y,
    //  mag_event.magnetic.z);
//    PushTxt(txt);
    txt[textctr] = 4;//code for htu
    textctr++;
    byte *bufbytetwelve = (byte *)&mag_event.magnetic.x;
    memcpy(&txt[textctr],&bufbytetwelve[0],sizeof(bufbytetwelve));
    textctr = textctr + sizeof(bufbytetwelve);
    byte *bufbytethirteen = (byte *)&mag_event.magnetic.y;
    memcpy(&txt[textctr],&bufbytethirteen[0],sizeof(bufbytethirteen));
    textctr = textctr + sizeof(bufbytethirteen);
    byte *bufbytefourteen = (byte *)&mag_event.magnetic.z;
    memcpy(&txt[textctr],&bufbytefourteen[0],sizeof(bufbytefourteen));
    textctr = textctr + sizeof(bufbytefourteen);
    txt[textctr] = 10;
    textctr++;  
    Serial.println(4);
    Serial.println(mag_event.magnetic.x);
    Serial.println(mag_event.magnetic.y);
    Serial.println(mag_event.magnetic.z);
    Serial.println();


    // Orientation (Easy to calculate later in Python - dont waste resources)
//#ifdef ORIENTATION
//    if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &xyz)) {
//      sprintf(txt,"{'MOG':{'Mox':%f,'Moy':%f,'Moz':%f}}\n",xyz.x,xyz.y,xyz.z);
//      PushTxt(txt);
//    }
//#endif
 // }
}


void PushAcl(int flg) { // Push the accelerometer and compass stuff
  sensors_event_t acl_event;
  sensors_vec_t xyz; 
Serial.print("going to get accelerator values");
  if ((flg) || ((acl_ok) && ((ppcnt % accelr_display_rate) == 0))) {
    acl.getEvent(&acl_event);

    // Meters per second squared

    //sprintf(txt,"{'ACL':{'Acx':%f,'Acy':%f,'Acz':%f}}\n",
      //acl_event.acceleration.x,
      //acl_event.acceleration.y,
      //acl_event.acceleration.z);
    //PushTxt(txt);
    txt[textctr] = 6;//code for htu
    textctr++;
    byte *bufbytefifteen = (byte *)&acl_event.acceleration.x;
    memcpy(&txt[textctr],&bufbytefifteen[0],sizeof(bufbytefifteen));
    textctr = textctr + sizeof(bufbytefifteen);
    byte *bufbytesixteen = (byte *)&acl_event.acceleration.y;
    memcpy(&txt[textctr],&bufbytesixteen[0],sizeof(bufbytesixteen));
    textctr = textctr + sizeof(bufbytesixteen);
    byte *bufbyteseventeen = (byte *)&acl_event.acceleration.z;
    memcpy(&txt[textctr],&bufbyteseventeen[0],sizeof(bufbyteseventeen));
    textctr = textctr + sizeof(bufbyteseventeen);
    txt[textctr] = 10;
    textctr++;  
    Serial.println(6);
    Serial.println(acl_event.acceleration.x);
    Serial.println(acl_event.acceleration.y);
    Serial.println(acl_event.acceleration.z);
    Serial.println();

    /*
    // Orientation (Easy to calculate later in Python - dont waste resources)
#ifdef ORIENTATION    
    if (dof.accelGetOrientation(&acl_event, &xyz)) {
      sprintf(txt,"{'AOL':{'Aox':%f,'Aoy':%f,'Aoz':%f}}\n",xyz.x,xyz.y,xyz.z);
      PushTxt(txt);
    }
#endif
*/

  }
}

void PushSts(int flg, int qsize, int missed) {
uint8_t res;

  if ((flg) || ((ppcnt % status_display_rate) == 0)) {
    //sprintf(txt,"{'STS':{'Qsz':%2d,'Mis':%2d,'Ter':%d,'Htu':%d,'Bmp':%d,'Acl':%d,'Mag':%d}}\n",
    //  qsize,missed,terr,htu_ok,bmp_ok,acl_ok,mag_ok);
    //PushTxt(txt);
    terr = 0;
    
    txt[textctr] = 10;//code for status
    textctr++;
    byte *bufbyteseventeenptone = (byte *)&qsize;
    memcpy(&txt[textctr],&bufbyteseventeenptone[0],sizeof(bufbyteseventeenptone));
    textctr = textctr + sizeof(bufbyteseventeenptone);
    byte *bufbyteseventeenpttwo = (byte *)&missed;
    memcpy(&txt[textctr],&bufbyteseventeenpttwo[0],sizeof(bufbyteseventeenpttwo));
    textctr = textctr + sizeof(bufbyteseventeenpttwo);
    byte *bufbyteeighteen = (byte *)&terr;
    memcpy(&txt[textctr],&bufbyteeighteen[0],sizeof(bufbyteeighteen));
    textctr = textctr + sizeof(bufbyteeighteen);
    
    if (htu_ok)
    {
      txt[textctr] = B00000001;
      textctr++;
    }
    else
    { 
      txt[textctr] = B00000000;
      textctr++;
    }
    
    //byte *bufbytenineteen = (byte *)&htu_ok;
    //memcpy(&txt[textctr],&bufbytenineteen[0],sizeof(bufbytenineteen));
    //textctr = textctr + sizeof(bufbytenineteen);
    
    if (bmp_ok)
    {
      txt[textctr] = B00000001;
      textctr++;
    }
    else
    { 
      txt[textctr] = B00000000;
      textctr++;
    }
    //byte *bufbytetwenty = (byte *)&bmp_ok;
    //memcpy(&txt[textctr],&bufbytetwenty[0],sizeof(bufbytetwenty));
    //textctr = textctr + sizeof(bufbytetwenty);
    
    if (acl_ok)
    {
      txt[textctr] = B00000001;
      textctr++;
    }
    else
    { 
      txt[textctr] = B00000000;
      textctr++;
    }
    
    //byte *bufbytetwentyone = (byte *)&acl_ok;
    //memcpy(&txt[textctr],&bufbytetwentyone[0],sizeof(bufbytetwentyone));
    //textctr = textctr + sizeof(bufbytetwentyone);

    if (mag_ok)
    {
      txt[textctr] = B00000001;
      textctr++;
    }
    else
    { 
      txt[textctr] = B00000000;
      textctr++;
    }    
    //byte *bufbytetwentytwo = (byte *)&mag_ok;
    //memcpy(&txt[textctr],&bufbytetwentytwo[0],sizeof(bufbytetwentytwo));
    //textctr = textctr + sizeof(bufbytetwentytwo);
    
    txt[textctr] = 10;
    textctr++;  
    
    Serial.println(10);
    
    Serial.println(qsize);
    Serial.println(missed);
    Serial.println(terr);
    Serial.println(htu_ok);
    Serial.println(bmp_ok);
    Serial.println(acl_ok);
//    Serial.println(bufbytetwentyone[3],DEC);
    Serial.println(mag_ok);
  //  Serial.println(bufbytetwentytwo[3],DEC);

    Serial.println(textctr);
    
    //Serial.println(sizeof(bufbytetwentyone));
    //Serial.println(sizeof(bufbytetwentytwo));
  
  }
}


void loop() {
textctr=0;
//Serial.println("alive");
//PushHtu(1);     // Push HTU temperature and humidity
//PushBmp(1);     // Push BMP temperature and barrometric altitude
//PushLoc(1);     // Push location latitude and longitude
//PushTim(1);     // Push timing data
  //PushMag(1);     // Push mago data
  //PushAcl(1);     // Push accelarometer data
  //PushSts(1,0,0);  // Push status
//    GetDateTime();      // Read the next date/time from the GPS chip
//if (textctr>0){
  SerialUSB.write(txt,textctr);
  //}
}


//status as of 9:30 Sunday night
//We've written a new protocol, transfering data at the bit level over USB for maximum efficiency and speed!
//writes in to a python script that justin's made
//output/input protocol working, using SerialUSB
//GPS not tested due to missing antenna, need to verify timestamp functionality and pps
//sensors working fine (baro, mag, accel, press, temp)
//Output for events:
//on trigger dump 300 samples per channel into buffer
//on pps dump buffer to SerialUSB.
//We can implement a character/byte passing system over SerialUSB if required, but it's very quick - so fingers crossed we don't need it.


