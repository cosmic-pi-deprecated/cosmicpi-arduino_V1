// This program collects up to PPS_EVENTS events each second into a double buffer.
// While the ISR fills one event buffer, the user space loop function reads from the other.
// Each PPS interrupt the read/write buffers are swapped over by the event ISR
// If any events are available in the read buffer the loop function puts them onto a queue
// along with the UTC time string where they get stored. Once there are at
// least event_display entries on the queue, the loop function outputs them over the serial
// line for processing. 

// In this version interrupts come from the accelerometer chip, if the acceleration exceeds
// the threshold (in meters/sec/sec) in any direction the chip interrupts and its logged to
// the serial output stream.
// The magnatometer chip if polled and if the deltas exceed the threshold a magnatometer 
// event is generated.

// Julian Lewis lewis.julian@gmail.com

#define FWVERS "13/December/2016 12:00"
#define CSVERS "V1"	// Output CSV version

// The output from this program is processed by a Python monitor on the other end of the
// serial line. There has to be mutual aggreement between this program and the monitor.

// Output strings
// All fields in all output strings conform to the json standard when the output_format
// global is set non zero, otherwise the output format is CSV

// Here is the list of all records where 'f' denotes float and 'i' denotes integer ...

// {'VER':{'Ver':%s}}
// Version string
//
// {'HTU':{'Tmh':f,'Hum':f}}
// HTU21DF record containing Tmh:temperature in C Hum:humidity percent
//
// {'BMP':{'Tmb':f,'Prs':f,'Alb':f}}
// BMP085 record containing Tmb:temperature Prs:pressure Alb:Barrometric altitude
//
// {'VIB':{'Vax':i,'Vcn':i}}
// Vibration record containing Vax:3 bit zyx direction mask Vcn:vibration count
// This record is always immediatly followed by 3 more records, TIM, ACL, and MAG
//
// {'MEV':{'Mev':i,'Met':f,'Mdx':f,'Mdy':f,'Mdz':f}}
// Magnetic event Mev:3 bit zyx status Met:Threshold Mdx:Delta x,Mdy:Delta y,Mdz:Delta z
//
// {'MAG':{'Mgx':f,'Mgy':f,'Mgz':f}}
// LSM303DLH magnatometer record containing Mgx:the x field strength Mgy:the y field Mgz:ther z field
//
// {'ACL':{'Acx':f,'Acy':f,'Acz':f}}
// LSM303DLH acclerometer record containing Acx:the x acceleration Acy:the y acceleration Acz:the z acceleration
// If this record immediatly follows a VIB record the fields were hardware latched when the g threshold was exceeded
//
// {'LOC':{'Lat':f,'Lon':f,'Alt':f}}
// GPS location record containing Lat:latitude in degrees Lon:longitude in degrees Alt:altitude in meters
//
// {'TIM':{'Upt':i,'Frq':i,'Sec':i}}
// Time record containing Upt:up time seconds Frq:counter frequency Sec:time string
//
// {'DTG':{'Yer':i, 'Mnt':i, 'Day':i}}
// GPS Date record containing Yer:Year Mnt:Month Day:Day in month
//
// {'STS':{'Qsz':i,'Mis':i,'Ter':i,'Tmx':i,'Htu':i,'Bmp':i,'Acl':i,'Mag':i, 'Gps':i, 'Adn':i, 'Gri':i, 'Eqt':i, 'Chm':i}}
// Status record containing Qsz:events on queue Mis:missed events Ter:buffer error Tmx:max buffer size reached
// Htu:status Bmp:status Acl:status Mag:status Gps:ststus 
// Adn:Number of samples per event Gri:Number of seconds between GPS reads Eqt:Event queue dump threshold Chm:Channel mask
//
// {'EVT':{'Evt':i,'Frq':i,'Tks':i,'Etm':f,'Adc':[[i,i,i,i,i,i,i,i][i,i,i,i,i,i,i,i]]}}
// Event record containing Evt:event number in second Frq:timer frequency Tks:ticks since last event in second 
// Etm:event time stamp to 100ns Adc:[[Channel 0 values][Channel 1 values]]
//
// {'HLP':{'Idn':i,'Nme':s,'Hlp':s}}
// Help for command ID number Idn with name nme and help text hlp
//
// {'TXT':{'Txt':s}}
// Text to be displayed
//
// {'BER':{'Ber':%d,'Adr':%s,'Reg':%s,'Bus':%d}}
// Bus error warning, Ber=error code, Adr=the hex address, Reg=the hex register, Bus=the bus number 0/1
//
// {'HPU':{'Ato':%s,'Hpu':%s,'Th0':%s,'Th1':%s,'Thr':%s,'Abr':%s}}
// HT power supply Ato:Thauto algorithm value Hpu:Manual value Thr:Threshold Abr:AB potentiometer bits
// If Abr is zero the thresholds are set automatically, Th0 and Th1 are the auto values for channel 0 and 1

// N.B. These records pass the data to a python monitor over the serial line. Python has awsome string handling and looks them up in
// associative arrays to build records of any arbitary format you want. So this is only the start of the story of record processing.
// N.B. Also some of these records are sent out at regular intervals and or when an event occurs.

// This program also accepts commands sent to it on the serial line.
// When a command arrives it is immediatly executed.

// About the hardware configuration: This firmware was originaly developed using Adafruit breakouts
// for the GPS, Accelerometer, Magnatometer, Barometer, Humidity and temperature. The cosmicpi main
// board (MB) implements these functions directly whith on board chips. However Adafruit breakouts
// can optionally replace the main board implementation which provides a pin compatible foot print
// on which the breakout can be inserted. Also you can build your own cosmic ray detector using the
// breakouts, and still use this firmware which supports both hardware configurations. The firmware
// detects the hardware and behaves according to whats installed. This has lead to some extra code.
// In particular the MB uses two I2C buses 0,1 and the Adafruit breakouts are on bus 0 mostly.

#include <time.h>
#include <Wire.h>
#include "LPS.h"	// Pololu's LPS library modified to use bus 1, https://github.com/pololu/lps-arduino

// Configuration constants

// The size of the one second event buffer
#define PPS_EVENTS 5	// The maximum number of events stored per second
#define ADC_BUF_LEN 32	// Maximum number of ADC values per event

// This is the event queue size
#define EVENT_QSIZE 32	// The number of events that can be queued for serial output

// Handle text buffer serial output overflow errors
// When the output buffer overflows due to data comming too fast, we just stop printing due
// to insufficient bandwidth or slow things down if HANDLE_OVERFLOW is set (not recomended)
// #define HANDLE_OVERFLOW

// This is the text ring buffer for real time output to serial line with interrupt on
#define TBLEN 8192	// Serial line output ring buffer size, 8K

// Define some output debug pins to monitor whats going on via an oscilloscope
#define PPS_PIN 11	// PPS (Pulse Per Second) and LED
#define EVT_PIN 12	// Cosmic ray event detected
#define FLG_PIN 13	// Debug flag

// Power pins for power on/off the breakouts after a reset
// The DUE makes a reset if the USB connection is restarted
// The AddaFruit breakouts loose it when this happens and the
// only way to recover them is a power cycle. This feature
// is redundant on the standard MB hardware configuration

#define POW_ONE 8	// High power 15ma
#define POW_TWO 9	// 15ma

// For siesmic event input
#define ACL_PIN 10	// Accelarometer INT1 interrupt pin

// Baud rates
#define SERIAL_BAUD_RATE 9600	// Serial line 
#define GPS_BAUD_RATE 9600	// GPS and Serial1 line

#define BLUE_LED_PIN 46
#define RED_LED_PIN 48

// Max5387 address pins

#define MAX1_PIN 35
#define MAX2_PIN 36
#define MAX3_PIN 37

// Count STRIGA and STRIGB interrupts

#define STRIGA_PIN 38
#define STRIGB_PIN 39

// Set up the pins to remap SPI by hand
const int SS_PIN   = 42; 
const int SCK_PIN  = 44;
const int MISO_PIN = 22;
const int MOSI_PIN = 43;

// Accelerometer/Magnatometer definitions for LMS303D (MB) and LSM303DLHC (Adafruit) chips
#define ACL_BUS_1_ADDR 0x1D	// LMS303D on the main board on i2c bus 1
#define ACL_BUS_0_ADDR 0x19	// LSM303DLHC on the Adafruit breakout on i2c bus 0
#define	MAG_BUS_0_ADDR 0x1E	// LSM303DLHC magnatometer
#define ACL_ID 0x49		// LMS303D ID register value
#define ACL_ID_REG 0x0F		// LMS303D ID register address
#define ACL_ADAFRUIT 1		// Used to say an Adafruite board detected
#define ACL_ON_MB 2		// Used to say LMS303D found on MB
#define ACL_CTRL_REG1_A 0x20	// Used to test for LSM303DLHC presence

// Leds  flag

int leds_on = 1;

// Hydrometer chip is always on bus 1 at the same address even for the adafruit breakout
#define HTU_BUS_1_ADDR 0x40

int acl_bus = 0;		// Bus number 0/1
int acl_id = 0;			// Which chip LSM303DLHC or LMS303D
int acl_ad = 0;			// Accelerometer address on the bus
int mag_ad = 0;			// Magnatometer address on the bus

// Barometric pressure and temperature
#define BMP_ON_MB 2
#define BMP_ADAFRUIT 1

int bmp_bus = 0;		// Barrometric pressure
int bmp_id = 0;			// 1=BMP085 2=LPS25H
int bmp_ad = 0;			// Address on bus
uint8_t bmp_ok = 0;

// GPS and time
boolean	gps_ok = false;		// Chip OK flag
boolean	time_ok = false;	// Time read from GPS OK
boolean pps_led = false;	// PPS Led 

// Forward refs
float HtuReadTemperature();
float HtuReadHumidity();
void  HtuReset();
uint8_t htu_ok = 0;

// Barrometer and temperature measurment LPS25H on bus 1 (MB)
LPS ps;

// Barrometer and temperature measurment BMP085 on bus 0 (Adafruit)
extern float BmpCalcTemp();
extern float BmpCalcPres();
extern float BmpCalcAlti();
extern char *BmpDebug();

// HT temperature automatic setting

void SetHtValue(int flg);

// Unique Arduino 128 bit ID code

void GetUid();
void PushUid(int flg);

// Control the output data rates by setting defaults, these values can be modified at run time
// via commands from the serial interface. Some output like position isn't supposed to be changing
// very fast if at all, so no need to clutter up the serial line with it. The Python monitor keeps
// the last sent values when it builds event messages to be sent over the internet to the server
// or logged to a file.

uint32_t uid_display_rate    = 30;	// Unique 128 bit display rate
uint32_t latlon_display_rate = 12;	// Display latitude and longitude each X seconds
uint32_t humtmp_display_rate = 12;	// Display humidity and HTU temperature each X seconds
uint32_t alttmp_display_rate = 12;	// Display altitude and BMP temperature each X seconds
uint32_t frqutc_display_rate = 1;	// Display frequency UTC time each X seconds
uint32_t frqdtg_display_rate = 10;	// Display frequency DTG GPS date each X seconds 
uint32_t status_display_rate = 4;	// Display status (UpTime, QueueSize, MissedEvents, HardwareOK)
uint32_t accelr_display_rate = 10;	// Display accelarometer x,y,z
uint32_t magnot_display_rate = 10;	// Display magnotometer data x,y,z
uint32_t gps_read_inc        = 0;	// How often to read the GPS (600 = every 10 minutes, 0 = always)
uint32_t events_display_size = 1;	// Display events after recieving X events
uint32_t adc_samples_per_evt = 8;	// Number of ADC samples per event
uint32_t channel_mask        = 3;	// Channel 1 and 2
uint32_t debug_gps           = 0;	// Debug print of GPS NMEA strings
uint32_t output_format       = 0;	// Output format 0=CSV else JSON, default CSV
uint32_t mag_poll_rate	     = 1;	// Magnetic polling ratei

// Siesmic and magnetic event trigger parameters

uint16_t accelr_event_threshold = 2;	// Trigger level for siesmic events +-2g full scale
uint16_t magnat_event_threshold = 300;  // Magnetic threshold +-4gauss full scale

// Commands can be sent over the serial line to configure the display rates or whatever

typedef enum {
	NOOP,	// No-operation
	LEDS,	// Leds On/Off
	VERS,	// Version string
	HELP,	// Help
	HTUX,	// Reset HTU chip
	HTUD,	// HUT display rate
	BMPD,	// BMP display rate
	LOCD,	// Location display rate
	TIMD,	// Timing display rate
	DTGD,	// Date display rate
	STSD,	// Status display rate
	EVQT,	// Event queue dump thresholdi
	UNID,   // Unique 128 bit ID display rate

	ACLD,	// Accelerometer display rate
	MAGD,	// Magnetometer display rate

	ACLT,	// Accelerometer event threshold
	MAGT,	// Magnatometer event threshold
	MPOL,	// Magnetic polling on
	GPRI,	// GPS read increment
	NADC,	// Number of ADC samples per event
	RBRK,	// Reset breakouts
	CHNS,	// Channels mask 0=none 1,2 or 3=both

	ABTS,	// Analogue board test
	GPID,	// Get GPS firmware ID
	GPPS,	// Test for PPS interrupts arriving
	DGPS,	// Debug GPS NMEA stringsi
	ACTS,	// Accelerometer test

	I2CS,	// I2C Bus scan 0,1
	D303,	// Dump LSM303 registers
	DHTU,	// Dump the HTU registers
	BMID,	// Get BMP chip ID

	WRPU,	// Write a value to the MAX1923 PU
	RCPU,	// Set recieve logic ON on the MAX1923 PU

	WRTH,	// Write thresholds to MAX5387
	ABTH,	// Select MAX5387 write pots
	STRG,	// STRIGA and STRIGB counters

	JSON,	// Set out put JSON 1, CSV 0

	CMDS };	// Command count

typedef struct {
	int   Id;		// Command ID number
	void  (*proc)(int arg);	// Function to call
	char *Name;		// Command name
	char *Help;		// Command help text
	int   Par;		// Command parameter flag
} CmdStruct;

#define CMD_ERROR 1		// General command failure error code
#define NO_SUCH_COMMAND 2
#define CMD_MAX_LEN 8
#define CMD_MAX_MSG 128

uint32_t cmd_result = 0;	// Last commands completion code
char     cmd_name[CMD_MAX_LEN];	// Last command name
char	 cmd_mesg[CMD_MAX_MSG]; // Last command message

// Command functions forward references 

void noop(int arg);
void leds(int arg);
void vers(int arg);
void help(int arg);
void htud(int arg);
void bmpd(int arg);
void locd(int arg);
void timd(int arg);
void dtgd(int arg);
void stsd(int arg);
void evqt(int arg);
void unid(int arg);
void acld(int arg);
void magd(int arg);
void aclt(int arg);
void magt(int arg);
void mpol(int arg);
void gpri(int arg);
void nadc(int arg);
void rbrk(int arg);
void chns(int arg);
void abts(int arg);
void gpid(int arg);
void gpps(int arg);
void dgps(int arg);
void acts(int arg);
void i2cs(int arg);
void d303(int arg);
void dhtu(int arg);
void bmid(int arg);
void wrpu(int arg);
void rcpu(int arg);
void wrth(int arg);
void abth(int arg);
void strg(int arg);
void json(int arg);

// Command table

CmdStruct cmd_table[CMDS] = {
	{ NOOP, noop, "NOOP", "Do nothing", 0 },
	{ LEDS, leds, "LEDS", "Leds on=1, off=0", 1 },
	{ VERS, vers, "VERS", "Version number", 0 },
	{ HELP, help, "HELP", "Display commands", 0 },
	{ HTUD, htud, "HTUD", "HTU Temperature-Humidity display rate", 1 },
	{ BMPD, bmpd, "BMPD", "BMP Temperature-Altitude display rate", 1 },
	{ LOCD, locd, "LOCD", "Location latitude-longitude display rate", 1 },
	{ TIMD, timd, "TIMD", "Timing uptime-frequency-utc display rate", 1 },
	{ DTGD, dtgd, "DTGD", "GPS Date display rate", 1 },
	{ STSD, stsd, "STSD", "Status info display rate", 1 },
	{ EVQT, evqt, "EVQT", "Event queue dump threshold", 1 },
	{ UNID, unid, "UNID", "Show Unique ID, arg = display rate", 1 },
	{ ACLD, acld, "ACLD", "Accelerometer display rate", 1 },
	{ MAGD, magd, "MAGD", "Magnatometer display rate", 1 },
	{ ACLT, aclt, "ACLT", "Accelerometer event trigger threshold", 1 },
	{ MAGT, magt, "MAGT", "Magnatometer event trigger threshold", 1 },
	{ MPOL, mpol, "MPOL", "Magnetic polling rate, 0=off else rate", 1 },
	{ GPRI, gpri, "GPRI", "GPS read increment in seconds", 1 },
	{ NADC, nadc, "NADC", "Number of ADC sampes tor read per event", 1 },
	{ RBRK, rbrk, "RBRK", "Reset power on=1/off=0 for breakouts", 1 },
	{ CHNS, chns, "CHNS", "Channel mask 0=none, 1,2 or 3=both", 1 },
	{ ABTS, abts, "ABTS", "Analogue Board test, 110=ADC Offsets, 120/121=SIPMs, 130=Vbias Threshold", 1 },
	{ GPID, gpid, "GPID", "Get GPS chip firmware ID", 1 },
	{ GPPS, gpps, "GPPS", "Test GPS is making PPS interrupts", 1 },
	{ DGPS, dgps, "DGPS", "Debug printing of GPS NMEA strings 0=off 1=on", 1 },
	{ ACTS, acts, "ACTS", "Accelerometer self test", 1 },
	{ I2CS, i2cs, "I2CS", "I2C Bus scan 0,1", 1 },
	{ D303, d303, "D303", "Dump LSM303 registers", 1 },
	{ DHTU, dhtu, "DHTU", "Dump HTU21D(F) registers", 1 },
	{ BMID, bmid, "BMID", "Get BMP chip type", 1 },
	{ WRPU,	wrpu, "WRPU", "Write a value to the MAX1923 PU", 1 },
	{ RCPU,	rcpu, "RCPU", "Recievie logic MAX1923 PU 0=just write, 1=setON, 2=setOFF", 1 },
	{ WRTH, wrth, "WRTH", "Write to the MAX5387 Threshold pots currently selected", 1 },
	{ ABTH, abth, "ABTH", "Select MAX5387 pots 1=A_ONLY, 2=B_ONLY, 3=A_AND_B", 1 },
	{ STRG, strg, "STRG", "Display strigA and strigB counters 1=Reset", 1 },
	{ JSON, json, "JSON", "Select output format JSON=1 or CSV=0 (default)", 1 }
};

#define CMDLEN 32
static char cmd[CMDLEN];		// Command input buffer
static int irdp=0, irdy=0, istp=0;	// Read, ready, stop
static char txtb[TBLEN];		// Text ring buffer
static uint32_t txtw = 0, txtr = 0, 	// Write and Read indexes
		tsze = 0, terr = 0,	// Buffer size and error code
		tmax = 0;		// The maximum size the buffer reached

typedef enum { TXT_NOERR=0, TXT_TOOBIG=1, TXT_OVERFL=2 } TxtErr;

#define TXTLEN 256
static char txt[TXTLEN];		// For writing to serial	

#define ADCHL (ADC_BUF_LEN * 6)		// "dddd," is 5 chars	 
static char adch0[ADCHL],adch1[ADCHL];	// ADC channel values strings

#define FREQ 42000000			// Clock frequency
#define MFRQ 40000000			// Sanity check frequency value

// Handle i2c bus errors

int bus_err = 0;
void PushBusError(int ber, uint8_t address, uint8_t reg, uint8_t bus) {
	bus_err = ber;
	if (ber) {
		if (output_format) 
			sprintf(txt,"{'BER':{'Ber':%d,'Adr':'0x%02X','Reg':'0x%02X','Bus':%d}}\n",
				ber,address,reg,bus);
		else sprintf(txt,"%s,BER,%d,%d,%d,%d\n",CSVERS,ber,address,reg,bus);

		PushTxt(txt);
	}
}

// I2C Bus IO routines for Bus 0 or 1

void BusWrite(uint8_t address, uint8_t reg, uint8_t value, uint8_t bus) {
	
	if (!address) return;

	if (bus) {
		Wire1.beginTransmission(address);
        	Wire1.write((uint8_t)reg);
		Wire1.write((uint8_t)value);
		PushBusError(Wire1.endTransmission(),address,reg,bus);
	} else {
		Wire.beginTransmission(address);
		Wire.write((uint8_t)reg);
		Wire.write((uint8_t)value);
		PushBusError(Wire.endTransmission(),address,reg,bus);
	}
}

uint8_t BusRead(uint8_t address, uint8_t reg, uint8_t bus) {
	uint8_t value;

	if (!address) return 0xFF;

	if (bus) {
		Wire1.beginTransmission(address);
		Wire1.write((uint8_t) reg);
		PushBusError(Wire1.endTransmission(),address,reg,bus);

		Wire1.requestFrom(address, (uint8_t) 1);
		value = Wire1.read();
		PushBusError(Wire1.endTransmission(),address,reg,bus);
	} else {
		Wire.beginTransmission(address);
		Wire.write((uint8_t) reg);
		PushBusError(Wire.endTransmission(),address,reg,bus);

		Wire.requestFrom(address, (uint8_t) 1);
		value = Wire.read();
		PushBusError(Wire.endTransmission(),address,reg,bus);
	}
	return value;
}

// Initialize the timer chips to measure time between the PPS pulses and the EVENT pulse
// The PPS enters pin D2, the PPS is forwarded accross an isolating diode to pin D5
// The event pulse is also connected to pin D5. So D5 sees the LOR of the PPS and the
// event, while D2 sees only the PPS. In this way we measure the frequency of the
// clock MCLK/2 each second on the first counter, and the time between EVENTs on the second
// I use a the unconnected timer block TC1 to make a PLL that is kept in phase by the PPS
// arrival in TC0 and which is loaded with the last measured PPS frequency. This PLL will
// take over the PPS generation if the real PPS goes missing.
// In this implementation the diode is implemented in software, see later

void TimersStart() {

        uint32_t config = 0;

	// Set up the power management controller for TC0 and TC2

        pmc_set_writeprotect(false);    // Enable write access to power management chip
        pmc_enable_periph_clk(ID_TC0);  // Turn on power for timer block 0 channel 0
        pmc_enable_periph_clk(ID_TC3);  // Turn on power for timer block 1 channel 0
        pmc_enable_periph_clk(ID_TC6);  // Turn on power for timer block 2 channel 0

	// Timer block 0 channel 0 is connected only to the PPS 
	// We set it up to load regester RA on each PPS and reset
	// So RA will contain the number of clock ticks between two PPS, this
	// value is the clock frequency and should be very stable +/- one tick

        config = TC_CMR_TCCLKS_TIMER_CLOCK1 |        	// Select fast clock MCK/2 = 42 MHz
                 TC_CMR_ETRGEDG_RISING |             	// External trigger rising edge on TIOA0
                 TC_CMR_ABETRG |                    	// Use the TIOA external input line
                 TC_CMR_LDRA_RISING;                 	// Latch counter value into RA

        TC_Configure(TC0, 0, config);                	// Configure channel 0 of TC0
        TC_Start(TC0, 0);                            	// Start timer running

        TC0->TC_CHANNEL[0].TC_IER =  TC_IER_LDRAS;   	// Enable the load AR channel 0 interrupt each PPS
        TC0->TC_CHANNEL[0].TC_IDR = ~TC_IER_LDRAS;   	// and disable the rest of the interrupt sources
        NVIC_EnableIRQ(TC0_IRQn);                    	// Enable interrupt handler for channel 0

	// Timer block 1 channel 0 is the PLL for when the GPS chip isn't providing the PPS
	// it has the frequency loaded in reg C and is triggered from the TC0 ISR

	config = TC_CMR_TCCLKS_TIMER_CLOCK1 |        	// Select fast clock MCK/2 = 42 MHz
		 TC_CMR_CPCTRG;				// Compare register C with count value

        TC_Configure(TC1, 0, config);                	// Configure channel 0 of TC1
        TC_SetRC(TC1, 0, FREQ);				// One second approx initial PLL value
	TC_Start(TC1, 0);                            	// Start timer running

        TC1->TC_CHANNEL[0].TC_IER =  TC_IER_CPCS;	// Enable the C register compare interrupt
        TC1->TC_CHANNEL[0].TC_IDR = ~TC_IER_CPCS;	// and disable the rest
        NVIC_EnableIRQ(TC3_IRQn);			// Enable interrupt handler for channel 0

	// Timer block 2 channel 0 is connected to the RAY event
	// It is kept in phase by the PPS comming from TC0 when the PPS arrives
	// or from TC1 when the PLL is active (This is the so called software diode logic)
 
        config = TC_CMR_TCCLKS_TIMER_CLOCK1 |        	// Select fast clock MCK/2 = 42 MHz
                 TC_CMR_ETRGEDG_RISING |             	// External trigger rising edge on TIOA1
                 TC_CMR_ABETRG |                     	// Use the TIOA external input line
                 TC_CMR_LDRA_RISING;                 	// Latch counter value into RA
 	
	TC_Configure(TC2, 0, config);                	// Configure channel 0 of TC2
	TC_Start(TC2, 0);			     	// Start timer running
 
	TC2->TC_CHANNEL[0].TC_IER =  TC_IER_LDRAS;   	// Enable the load AR channel 0 interrupt each PPS
	TC2->TC_CHANNEL[0].TC_IDR = ~TC_IER_LDRAS;   	// and disable the rest of the interrupt sources
	NVIC_EnableIRQ(TC6_IRQn);                    	// Enable interrupt handler for channel 0

	// Set up the PIO controller to route input pins for TC0 and TC2

	PIO_Configure(PIOC,PIO_INPUT,
		      PIO_PB25B_TIOA0,	// D2 Input	
		      PIO_DEFAULT);

	PIO_Configure(PIOC,PIO_INPUT,
		      PIO_PC25B_TIOA6,	// D5 Input
		      PIO_DEFAULT);
}

// Timer chip interrupt handlers try to get time stamps to within 4 system clock ticks

static uint32_t displ = 0;	// Display values in loop

static uint32_t	rega0 = FREQ, 	// RA reg
		stsr0 = 0,	// Interrupt status register
		ppcnt = 0,	// PPS count
		delcn = 0;	// Synthetic PPS ms

static uint32_t	rega1, stsr1 = 0;

static uint32_t stsr2 = 0;

boolean pll_flag = false;	

int old_ra = 0;
int new_ra = 0;
#define DEAD_TIME 42000	// 1ms

// Handle the PPS interrupt in counter block 0 ISR

void TC0_Handler() {

	// In principal we could connect a diode
	// to pass on the PPS to counter blocks 1 & 2. However for some unknown
	// reason this pulls down the PPS voltage level to less than 1V and
	// the trigger becomes unreliable !! 
	// In any case the PPS is 100ms wide !! Introducing a blind spot when
	// the diode creates the OR of the event trigger and the PPS.
	// So this is a software diode

	TC2->TC_CHANNEL[0].TC_CCR = TC_CCR_SWTRG; // Forward PPS to counter block 2
	TC1->TC_CHANNEL[0].TC_CCR = TC_CCR_SWTRG; // Forward PPS to counter block 1

	rega0 = TC0->TC_CHANNEL[0].TC_RA;	// Read the RA reg (PPS period)
	stsr0 = TC_GetStatus(TC0, 0); 		// Read status and clear load bits

	if (rega0 < MFRQ)			// Sanity check against noise
		rega0 = FREQ;			// Use nominal value
	
        TC_SetRC(TC1, 0, rega0);		// Set the PLL count to what we just counted

	SwapBufs();				// Every PPS swap the read/write buffers
	ppcnt++;				// PPS count
	displ = 1;				// Display stuff in the loop
	gps_ok = true;				// Its OK because we got a PPS	
	pll_flag = true;			// Inhibit PLL, dont take over PPS arrived

	old_ra = 0;				// Dead time counters
	new_ra = 0;
	
	IncDateTime();				// Next second

	if (leds_on) {
		if (pps_led) {		
			digitalWrite(PPS_PIN,HIGH);
			pps_led = false;
		} else {
			digitalWrite(PPS_PIN,LOW);
			pps_led = true;
		}
	}
}

// Handle PLL interrupts
// When/If the PPS goes missing due to a lost lock we carry on with the last measured
// value for the second from TC0

void TC3_Handler() {

	stsr2 = TC_GetStatus(TC1, 0); 		// Read status and clear interrupt
#if FLG_PIN
	digitalWrite(FLG_PIN,HIGH);		// Flag set (for debug)
	digitalWrite(FLG_PIN,LOW);
#endif

	if (pll_flag == false) {		// Only take over when no PPS

		TC2->TC_CHANNEL[0].TC_CCR = TC_CCR_SWTRG; // Forward PPS to counter block 2
		SwapBufs();				// Every PPS swap the read/write buffers
		ppcnt++;				// PPS count
		displ = 1;				// Display stuff in the loop
		gps_ok = false;				// PPS missing

		IncDateTime();				// Next second
	}
	pll_flag = false;				// Take over until PPS comes back
}

// We need a double buffer, one is being written by the ISR while
// the other is read from user space within one second.

struct Event {
	uint16_t Ch0[ADC_BUF_LEN];	// ADC channel 0 values
	uint16_t Ch1[ADC_BUF_LEN];	// ADC channel 1 values
	uint32_t Tks;			// Time since last event in ticks
};

static struct Event b1[PPS_EVENTS];	// Event ticks buffeer
static struct Event b2[PPS_EVENTS];	// Event ticks buffer
static struct Event *wbuf = b1;		// Write event buffer pointer and its index
static struct Event *rbuf = b2;		// Read event buffer pointer and its index
static int ridx, widx;

// We also need a time value for the current and previous second

#define DATE_TIME_LEN 9

static char t1[DATE_TIME_LEN];		// Date time buffer text string
static char t2[DATE_TIME_LEN];		
static char *wdtm = t1;			// Write date/time pointer
static char *rdtm = t2;			// Read date/time pointer

// Swap read write event buffers and indexes along with their time strings
// each second, so we have the current and previous second numbers

int inc_ht_flg = 0;	// Too many interrupts, increment the HT control
int dec_ht_flg = 0;	// No interrupts, decrement the HT control

void SwapBufs() {
	struct Event *tbuf;			// Temp event buf pointer
	char *tdtm;				// Temp date/time string pointer

	if (!widx) dec_ht_flg++;		// No interrupts: dec the HT value

	tbuf = rbuf; rbuf = wbuf; wbuf = tbuf;	// Swap write with read buffer
	ridx = widx; widx = 0;			// Write count to read, reset the write count
	tdtm = rdtm; rdtm = wdtm; wdtm = tdtm;	// And swap asociated buffer date/time
}

// Handle isolated PPS (via diode) LOR with the Event
// The diode is needed to block Event pulses getting back to TC0
// LOR means Logical inclusive OR
// Now we are using the software diode implementation

void TC6_Handler() {

	if (widx < PPS_EVENTS) {	// Up to PPS_EVENTS stored per PPS
			
		// Read the latched tick count getting the event time
		// and then pull the ADC pipe line

		new_ra = TC2->TC_CHANNEL[0].TC_RA;
		if (new_ra - old_ra > DEAD_TIME) {
			old_ra = new_ra;
			wbuf[widx].Tks = new_ra;
			AdcPullData(&wbuf[widx]);
			widx++;
			if (leds_on)
				digitalWrite(EVT_PIN,HIGH);	// Event LEP on, off in loop()

		} // else inc_ht_flg++;
	} else
		inc_ht_flg++;

	rega1 = TC2->TC_CHANNEL[0].TC_RA;	// Read the RA on channel 1 (PPS period)
	stsr1 = TC_GetStatus(TC2, 0); 		// Read status clear load bits

}

// Discover the hardware configuration 

void GetAclId() {
	uint8_t id;

	if (acl_id) return;

	acl_bus = 1;
	id = BusRead(ACL_BUS_1_ADDR,ACL_ID_REG,1);	
	if (id == ACL_ID) { 
		acl_id = ACL_ON_MB;
		acl_ad = ACL_BUS_1_ADDR;
		mag_ad = ACL_BUS_1_ADDR;
		return;
	}

	acl_bus = 0;
	BusWrite(ACL_BUS_0_ADDR, ACL_CTRL_REG1_A, 0x57, 0);
        id = BusRead(ACL_BUS_0_ADDR, ACL_CTRL_REG1_A, 0);
        if (id == 0x57) {
		acl_id = ACL_ADAFRUIT;
		acl_ad = ACL_BUS_0_ADDR;
		mag_ad = MAG_BUS_0_ADDR;
		return;
	}

	acl_bus = 0;
	acl_id = 0;
	acl_ad = 0;
	mag_ad = 0;
}

// The following setup makes the accelarometer compare G-forces against a threshold value, and
// latch the output registers until they are read. To avoid excessive interrupt rates the high
// pass filter has been configured to keep the frequency low. 

void AclSetup() {

	GetAclId();

	if (acl_id == ACL_ADAFRUIT) {
		AclAdaSetup();
		return;
	}
	if (acl_id == ACL_ON_MB) {
		AclMbSetup();
		return;
	}
}

// Mainboard magnatometer and accelerometer setup
// LSM303D chip

void AclMbSetup() {
	uint8_t tmp, val;

#define FIFO_EN 0x40
#define HPIS1   0x02
#define HPIS2	0x01

	val = FIFO_EN | HPIS1;
	// BusWrite(acl_ad, 0x1F, val, 1); // CTRL0

#define MXYZEN (0x7 << 5)	// Enable XYZ interrupt detection
#define MIELEN 0x1		// Enable

	val = MXYZEN | MIELEN;
	// BusWrite(acl_ad, 0x12, val, 1); // INT_CTRL_M

	val = magnat_event_threshold >> 8;	// High
	// BusWrite(acl_ad, 0x15, val, 1);		// INT_THS_H_M
	val = magnat_event_threshold & 0xFF;	// Low
	// BusWrite(acl_ad, 0x14, val, 1);		// INT_THS_L_M

#define AXYXEN 0x7
#define A50Hz (0x5 << 4)

	val = AXYXEN | A50Hz;		// x,y,z enable and 50 Hz
	BusWrite(acl_ad, 0x20, val, 1);	// CTRL1

#define GRANGE 2.0
#define AFS2G (0x00 << 3)

	val = AFS2G;			// +/- 2g Full scale
	BusWrite(acl_ad,0x21,val, 1);	// CTRL2

#define INT1_DRDY_A 0x4
#define INT1_IG1 0x20

	val = INT1_IG1;			// Inertial interrupts on INT1 enabled
	BusWrite(acl_ad, 0x22, val, 1);	// CTRL3

#define INT2_IGM 0x10
#define INT2_DRDY_M 0x4

	val = INT2_IGM | INT2_DRDY_M;
	// BusWrite(acl_ad, 0x23, val, 1);

#define LIR1 0x1	// Latch interrupt 1
#define MODR (0x4 << 2)	// 50Hz
#define MRES (0x3 << 5) // High resolution
#define TEMPEN 0x80	// Temperature enabled

	val = LIR1 | MODR | MRES | TEMPEN;		
	BusWrite(acl_ad, 0x24, val, 1);	// CTRL5

#define MFS (0x1 << 5)		// +/- 4 Gauss full scale
	
	val = MFS;
	BusWrite(acl_ad, 0x25, val, 1); // CTRL6

#define MD 0x0		// Continuous mode

	val = MD;
	BusWrite(acl_ad, 0x26, val, 1); // CTRL7

#define AI6D 0x80
#define ZHIE 0x20
#define YHIE 0x08
#define XHIE 0x02

	val = AI6D | XHIE | YHIE | ZHIE;	// Interrupt on high x,y,z
	BusWrite(acl_ad, 0x30, val, 1);		// IG_CFG1

	val = accelr_event_threshold & 0x7F;
	BusWrite(acl_ad, 0x32, val, 1);		// Ineterial threshold

	val = 1;			// Interrupt duration
	BusWrite(acl_ad, 0x33, val, 1);	// IG1_DUR1
	 
	val = BusRead(acl_ad, 0x31, 1);	// IG_SRC1 read and clear interrupts

	attachInterrupt(digitalPinToInterrupt(30),Acl_ISR,RISING);
	attachInterrupt(digitalPinToInterrupt(29),Mag_ISR,RISING);
}

// Adafruit accelerometer setup
// LSM303DLH Chip

void AclAdaSetup() {

	uint8_t tmp, val;

#define PMD 0x20	// Normal power mode (PM0=1,PM1=0:Normal)
#define DRT 0x00	// Data rate 50 Hz (0x08 = 100Hz)
#define AEN 0x07	// XYZ Enabled

	val = PMD | DRT | AEN;
	BusWrite(acl_ad, 0x20, val, 0);

#define HPE1 0x04	// High pass filter Int 1 on
#define HPCF 0x03       // High pass cut off frequency

	val = HPE1 | HPCF;
	BusWrite(acl_ad, 0x21, val, 0);

#define ALIR1 0x06	// Latch Int1 bit Data ready

#define IHL_OD 0xC0	// Interrupt active low, open drain (Argh !!!)

	val = ALIR1 | IHL_OD;
	BusWrite(acl_ad, 0x22, val, 0);

#define BDU_FS 0x80	// Block data and scale +-2g

	val = BDU_FS;
	BusWrite(acl_ad, 0x23, val, 0);

#define XYZ_HI 0x2A	// Hi values ZHIE YHIE XHIE
#define AOI_6D 0x00	// 0xC0 would enable 6 directions
	
	val = XYZ_HI | AOI_6D;
	BusWrite(acl_ad, 0x30, val, 0);

	val = accelr_event_threshold & 0x7F;
	BusWrite(acl_ad, 0x32, val, 0);

	// The chip make very wide pulses (100ms), the values on the rising and
	// falling edges are different !

	attachInterrupt(digitalPinToInterrupt(ACL_PIN),Acl_ISR,RISING);
}

// This Accelerometer ISR

static uint32_t accl_icount = 0, accl_flag = 0;

void Acl_ISR() {

	if ( (accl_flag=AclReadStatus()) ) {
		accl_icount++;
		PushVib();
	}
}

// Magnatometer ISR

static uint32_t magn_icount = 0, magn_flag = 0;

void Mag_ISR() {

	magn_flag=MagReadStatus();
	magn_icount++;
	if (magn_flag) PushMev();
}

// Read accelerometer status
// This just reads the interrupt source INT1 and the overrun status.
// It returns 1 bit for X, Y, or Z (0..7) if the threshold value is exceeded.
// This determins if the board is being shaken - Earth quake - or other reason

static uint8_t acl_src = 0;

uint8_t AclReadStatus() {
	uint8_t rval;

	acl_src = BusRead(acl_ad, 0x31, acl_bus);	// IG_SRC1 read and clear interrupts

#define IA 0x40

#define ZH 0x20	// Z High
#define YH 0x08 // Y High
#define XH 0x02 // X High

	rval = 0;
	if (acl_src & IA) {
		if (acl_src & ZH) rval |= 4;
		if (acl_src & YH) rval |= 2;
		if (acl_src & XH) rval |= 1;
	}
	return rval;
}

// Read magnatometer status

uint8_t MagReadStatus() {
	acl_src = BusRead(acl_ad, 0x35, 1);	// IG_SRC2 read and clear interrupts
	return BusRead(acl_ad, 0x13, 1);	// INT_SRC_M
}

// Read accelerometer 

short acl_x=0, acl_y=0, acl_z=0;
float acl_fx=0.0, acl_fy=0.0, acl_fz=0.0;

#define GEARTH 9.80665
#define Gg (9.80665 * 0.001)

#define ACL_FS 2.0 // +-2g 16 bit 2's compliment

// Convert to meters per sec per sec

float AclToMs2(short val) {
	return (ACL_FS * GEARTH) * ((float) val / (float) 0x7FFF);
}

void AclReadData() {
	uint8_t xlo,xhi,ylo,yhi,zlo,zhi;

	xlo=BusRead(acl_ad, 0x28, acl_bus);
	xhi=BusRead(acl_ad, 0x29, acl_bus);
	ylo=BusRead(acl_ad, 0x2A, acl_bus);
	yhi=BusRead(acl_ad, 0x2B, acl_bus);
	zlo=BusRead(acl_ad, 0x2C, acl_bus);
	zhi=BusRead(acl_ad, 0x2D, acl_bus);

	if (acl_id == ACL_ADAFRUIT) { 
		acl_x = (xhi<<8 | xlo) >> 4;
		acl_y = (yhi<<8 | ylo) >> 4;
		acl_z = (zhi<<8 | zlo) >> 4;
	
		acl_fx = (float) acl_x * Gg;
		acl_fy = (float) acl_y * Gg;
		acl_fz = (float) acl_z * Gg;
	} else {
		acl_x = (xhi<<8 | xlo);
		acl_y = (yhi<<8 | ylo);
		acl_z = (zhi<<8 | zlo);

		acl_fx = AclToMs2(acl_x);
		acl_fy = AclToMs2(acl_y);
		acl_fz = AclToMs2(acl_z);
	}
}

// Simple read of magnatometer data

short mag_x=0, mag_y=0, mag_z=0;
float mag_fx=0.0, mag_fy=0.0, mag_fz=0.0;

#define MAGNETIC_FS 4.0	// Full scale Gauss 16 bit 2's compliment

float MagToGauss(short val) {
	return (MAGNETIC_FS * (float) val) / (float) 0x7FFF;
}

void MagConvData() {
	mag_fx = MagToGauss(mag_x);
	mag_fy = MagToGauss(mag_y);
	mag_fz = MagToGauss(mag_z);
}

void MagReadData() {
	uint8_t xlo,xhi,ylo,yhi,zlo,zhi;
	
	if (acl_id == ACL_ADAFRUIT) {
		xlo=BusRead(mag_ad,0x3,0);
		xhi=BusRead(mag_ad,0x4,0);
		ylo=BusRead(mag_ad,0x5,0);
		yhi=BusRead(mag_ad,0x6,0);
		zlo=BusRead(mag_ad,0x7,0);
		zhi=BusRead(mag_ad,0x8,0);
	}
		
	if (acl_id == ACL_ON_MB) {
		xlo=BusRead(acl_ad,0x8,1);
		xhi=BusRead(acl_ad,0x9,1);
		ylo=BusRead(acl_ad,0xA,1);
		yhi=BusRead(acl_ad,0xB,1);
		zlo=BusRead(acl_ad,0xC,1);
		zhi=BusRead(acl_ad,0xD,1);
	}

	mag_x = (xhi<<8 | xlo);
	mag_y = (yhi<<8 | ylo);
	mag_z = (zhi<<8 | zlo);
}

// Poll the magnatometer data for events

short omag_x=0, omag_y=0, omag_z=0;	// Old values
short dmag_x=0, dmag_y=0, dmag_z=0;	// Delta values
uint8_t mev_flg = 0;			// Magnetic event flag z,y,x bits

void MagPoll() {

	mev_flg = 0;	// No z,x,y data

	dmag_x = omag_x - mag_x;
	dmag_y = omag_y - mag_y;
	dmag_z = omag_z - mag_z;

	if (abs(dmag_x) > magnat_event_threshold) mev_flg |= 0x1;
	if (abs(dmag_y) > magnat_event_threshold) mev_flg |= 0x2;
	if (abs(dmag_z) > magnat_event_threshold) mev_flg |= 0x4;
	
	omag_x = mag_x;
	omag_y = mag_y;
	omag_z = mag_z;

	if ((mev_flg) && (mag_poll_rate) && (acl_id) && ((ppcnt % mag_poll_rate) == 0)) PushMev();
}

// Set up the ADC channels

void AdcSetup() {
	REG_ADC_MR = 0x10380080;	// Free run as fast as you can
	REG_ADC_CHER = 3;		// Channels 0 and 1
	REG_ADC_CR = 2;			// Start
}

// Pull all data (16 values) from the ADC into a buffer

uint16_t avc0 = 0;
uint16_t avc1 = 0;

uint8_t AdcPullData(struct Event *b) {
	
	int i, a0=0, a1=0;

	for (i=0; i<adc_samples_per_evt; i++) {		// For all in ADC pipeline
		if (channel_mask & 0x01) {
			while((ADC->ADC_ISR & 0x01)==0);	// Wait for channel 0 (2.5us)
			b->Ch0[i] = (uint16_t) ADC->ADC_CDR[0];	// Read ch 0
			a0 += b->Ch0[i];			// Average for threshold setting
		}
		if (channel_mask & 0x02) {
			while((ADC->ADC_ISR & 0x02)==0);	// Wait for channel 1 (2.5us)
			b->Ch1[i] = (uint16_t) ADC->ADC_CDR[1];	// Read ch 1
			a1 += b->Ch1[i];			// Average for threshold setting
		}
	}

	a0 = a0/adc_samples_per_evt;
	if (avc0) avc0 = (avc0+a0)/2;				// Running average
	else      avc0 = a0;

	a1 = a1/adc_samples_per_evt;
	if (avc1) avc1 = (avc1+a1)/2;				// Running average
	else	  avc1 = a1;
}

// Increment date time by one second when not using the GPS

int year=0, month=0,  day=0;
int hour=0, minute=0, second=0;
float latitude = 0.0, longitude = 0.0, altitude = 0.0;

void IncDateTime() {

	if (++second >= 60) {
		second = 0;
		if (++minute >= 60) {
			minute = 0;
			if (++hour >= 24) {
				hour = 0;
			}
		}
	}
	sprintf(wdtm,"%02d%02d%02d",hour,minute,second);
}

// This is the nmea data string from the GPS chip

#define GPS_STRING_LEN 256
static char gps_string[GPS_STRING_LEN + 1];

#define QUECTEL76_GPS 76
#define ADAFRUIT_GPS 60

static int gps_id = 0;
void GetGpsId() {

	if (strstr(gps_string,"Quectel-L76"))
		gps_id = QUECTEL76_GPS;
	else if (strstr(gps_string,"PA6H"))
		gps_id = ADAFRUIT_GPS;
}

// WARNING: One up the spout !!
// The GPS chip puts the next nmea string in its output buffer
// only if its been read, IE its empty.
// So if you read infrequently the string in the buffer is old and
// has the WRONG time !!! The string lies around like a bullet in
// the breach waiting for some mug.

boolean ReadGpsString() {

	int i = 0;

	if (!gps_ok) 
		return false;

	while (Serial1.available()) {
		if (i < GPS_STRING_LEN) {
			gps_string[i++] = (char) Serial1.read();
			gps_string[i] = 0;
		} else i++;
	}

	if (i != 0) {
		if (!gps_id) GetGpsId();
		if (debug_gps) {
			sprintf(txt,"%s\n",gps_string);
			PushTxt(txt);
		}
		return true;
	}
	return false;
}

// Get sub string par

int GetSubPar(char *cp, int len) {
	char tbuf[16], *ep;
	strncpy(tbuf,cp,len);
	tbuf[len] = 0;
	return strtoul(tbuf,&ep,10);
}

// Parse the GPS NMEA string

#define LOCPARS 9
#define DATPARS 6

int date_ok  = 0;
int send_gga = 0;

boolean ParsGpsString() {
	char *pars[LOCPARS], *cp, *ep;
	int i;

	if ((cp=strstr(gps_string,"$GPZDA"))) {
		for (i=0; i<DATPARS; i++) {
			cp = index(cp,',');
			if (cp) {
				*cp = 0;
				pars[i] = ++cp;
			} else  pars[i] = NULL;
		}
		
		cp = pars[1];
		day = strtoul(cp,&ep,10);
		cp = pars[2];
		month = strtoul(cp,&ep,10);
		cp = pars[3];
		year = strtoul(cp,&ep,10);

		date_ok = 1;
	}

	if ((cp=strstr(gps_string,"$GPGGA"))) {
		for (i=0; i<LOCPARS; i++) {
			cp = index(cp,','); 
			if (cp) {
				*cp = 0;
				pars[i] = ++cp;
			} else  pars[i] = NULL;
		}
		
		// pars[0] contains "hhmmss"

		cp = pars[0];
		hour = GetSubPar(cp,2);

		cp = &pars[0][2];
		minute = GetSubPar(cp,2);

		cp = &pars[0][4];
		second = GetSubPar(cp,2);
		
		// lat, lon, alt are all floats at indexes 1,3,8

		cp = pars[1];
		latitude = strtof(cp,&ep);
		
		cp = pars[3];
		longitude = strtof(cp,&ep);

		cp = pars[8];
		altitude = strtof(cp,&ep);

		if (debug_gps) {
			for (i=0; i<LOCPARS; i++) {
				sprintf(txt,"%d:%s ",i,pars[i]);
				PushTxt(txt);
			}
			sprintf(txt,"-GPGGA\n");
		}
		return true;
	}
	return false;
}

// Get the date time from GPS

boolean GpsDateTime() {

	if ((ReadGpsString()) && (ParsGpsString())) {

		sprintf(wdtm,"%02d%02d%02d",hour,minute,second);
		time_ok = true;
		if ((hour==23) && (minute==59) && (second==59)) date_ok = 0;
		return true;
	}
	return false;
}

// Get the date time either from GPS each gps_read_inc

uint32_t nxtdtr = 0;	// Next DateTime read second number

void GetDateTime() {

	if (gps_ok) {						// If the GPS is up
		if (!time_ok) { 				// If I havn't read it ever
			if (GpsDateTime()) {			// read GPS time and check its OK
				nxtdtr = ppcnt + gps_read_inc;	// Next time to read
				return;
			}
		} else {
			if (gps_read_inc) {			// Guaranteed to be 0 or greater than 2
				if (ppcnt == nxtdtr) {		// Clean out buffer ?
					ReadGpsString();	// One up the spout, get rid of it
				}
			}
			if (ppcnt > nxtdtr) {			// Time to read GPS again ?
				if (GpsDateTime()) {
					nxtdtr = ppcnt + gps_read_inc;
					return;
				}
			}
		}
	}
}

// Implement queue access mechanism for events, each second the user space (loop) copies
// any events it has read onto the queue

struct EventBuf {
	char		DateTime[DATE_TIME_LEN];	// The date and time string
	uint32_t	Frequency;			// The current clock frequency
	uint32_t	Ticks;				// The number of ticks since the last event or PPS if none
	uint16_t	Ch0[ADC_BUF_LEN];		// ADC channel 0 values
	uint16_t	Ch1[ADC_BUF_LEN];		// ADC channel 1 values
	uint8_t		Count;				// The number of events since the PPS
};

typedef struct {
	uint8_t		Size;				// Current size of the queue
	uint8_t		RdPtr;				// Read pointer
	uint8_t		WrPtr;				// Write pointer
	uint8_t		Missed;				// Missed events counter due to overflow
	struct EventBuf	Events[EVENT_QSIZE];		// Queued events 
} EventQueue;

static EventQueue event_queue;

// Put an event in an EventBuf on the queue, if the queue is full then the oldest event
// is thrown away and the "missed" event count is incremented

uint8_t PutQueue(struct EventBuf *ebuf) {

	EventQueue *q = &event_queue;

	q->Events[q->WrPtr] = *ebuf;		// Write event to the queue
	q->WrPtr = (q->WrPtr + 1) % EVENT_QSIZE;// Increment the write pointer
	if (q->Size < EVENT_QSIZE) q->Size++;	// If we are overwriting old enties that havnt been read
	else {
		q->Missed++;					// Say we missed some events
		q->RdPtr = (q->RdPtr + 1) % EVENT_QSIZE;	// and throw the oldest event away	
	}
	return q->Missed;
}

// Pop an event off the queue, if the queue is empty nothing happens
// the queue size is zero when the queue is empty.
// If the pop resulted in an event return 1 else 0

uint8_t PopQueue(struct EventBuf *ebuf) {	// Points to where the caller wants the event stored

	EventQueue *q = &event_queue;

	if (q->Size) {
		*ebuf = q->Events[q->RdPtr];
		q->RdPtr = (q->RdPtr + 1) % EVENT_QSIZE;
		q->Size--;
		return 1;
	}
	return 0; 
}

// Get the size of the queue

uint8_t SzeQueue() {

	EventQueue *q = &event_queue;

	return q->Size;
}
 
// Initialize the queue

void InitQueue() {

	EventQueue *q = &event_queue;

	q->Size = 0;
	q->RdPtr = 0;
	q->WrPtr = 0;
	q->Missed = 0;
}

// GPS setup

void GpsSetup() {

#define RMCGGA		"$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"	// RMCGGA
#define RMCZDA		"$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0*29"	// RMCZDA
#define FMWVERS 	"$PMTK605*31"  						// PMTK_Q_RELEASE
#define NORMAL  	"$PMTK220,1000*1F"					// PMTK_SET_NMEA_UPDATE_1HZ
#define NOANTENNA	"$PGCMD,33,0*6D" 					// PGCMD_NOAN

	Serial1.begin(9600);
	Serial1.println(NOANTENNA);
	Serial1.println(RMCGGA);
	Serial1.println(NORMAL);
	Serial1.println(FMWVERS);
}

// Count interrupts from STRIGA and STRIGB

long striga = 0;
void StrigA_ISR() {
	striga++;
}

long strigb = 0;
void StrigB_ISR() {
	strigb++;
}

void Strig_setup() {
	pinMode(STRIGA_PIN, INPUT);
	pinMode(STRIGB_PIN, INPUT);
	attachInterrupt(digitalPinToInterrupt(STRIGA_PIN),StrigA_ISR,RISING);
	attachInterrupt(digitalPinToInterrupt(STRIGB_PIN),StrigB_ISR,RISING);
}

// Arduino setup function, initialize hardware and software
// This is the first function to be called when the sketch is started

void setup() {
	
	GetUid();			// Read unique 128 bit ID from memory

	Wire.begin();
	Wire1.begin();

#if FLG_PIN
	pinMode(FLG_PIN, OUTPUT);	// Pin for the ppsfl flag for debug
#endif
	pinMode(EVT_PIN, OUTPUT);	// Pin for the cosmic ray event 
	pinMode(PPS_PIN, OUTPUT);	// Pin for the PPS (LED pin)

	pinMode(POW_ONE, OUTPUT);	// Breakout power pin
	pinMode(POW_TWO, OUTPUT);	// ditto

	pinMode(BLUE_LED_PIN, OUTPUT);	
	pinMode(RED_LED_PIN, OUTPUT);

	pinMode(MAX1_PIN, OUTPUT);
	pinMode(MAX2_PIN, OUTPUT);
	pinMode(MAX3_PIN, OUTPUT);

	digitalWrite(MAX1_PIN, LOW);
	digitalWrite(MAX2_PIN, LOW);
	digitalWrite(MAX3_PIN, LOW);

	Serial.begin(SERIAL_BAUD_RATE);	// Start the serial line
	Serial1.begin(GPS_BAUD_RATE);	// and the second

	GpsSetup();

	InitQueue();			// Reset queue pointers, missed count, and size

	strcpy(rdtm,"");		// Set initial value for date/time
	strcpy(wdtm,"");

	digitalWrite(SS, HIGH);  // Start with SS high
	pinMode(SS_PIN, OUTPUT);
	pinMode(SCK_PIN, OUTPUT);
	pinMode(MISO_PIN, INPUT); //this is the avalanche pin, not implemented yet
	pinMode(MOSI_PIN, OUTPUT);

	// Power OFF/ON the breakouts

	digitalWrite(PPS_PIN,LOW);
	digitalWrite(EVT_PIN,HIGH);
	rbrk(0);
	rbrk(1);
	digitalWrite(PPS_PIN,HIGH);
	digitalWrite(EVT_PIN,LOW);

	// Initialize breakouts

	HtuReset();
	AclSetup();
	AdcSetup();
	
	GetBmpId();
	if (bmp_id == BMP_ON_MB) {
		if (!ps.init()) bmp_ok = 0;
		else ps.enableDefault();
	} else if (bmp_id == BMP_ADAFRUIT) {
		BmpCalcPres();
	}	

	SetHtValue(1);	// Set the High Tension
	TimersStart();	// Start timers
	PushUid(1);	// Push UID 128 bit codei

	Strig_setup();	// Counter trigger interrupts
}

// These two routines are needed because the Serial.print method prints without using interrupts.
// Calls to Serial.print block interrupts and use a wait in kernel space causing all ISRs to
// be blocked and hence we could miss some timer interrupts.
// To avoid this problem call PushTxt to have stuff delivered to the serial line, PushTxt simply
// stores your text for future print out by PutChar. The PutChar routine removes one character
// from the stored text each time its called. By placing a call to PutChar in the outermost loop
// of the Arduino loop function, then for each loop one character is printed, avoiding blocking
// of interrupts and vastly improving the loops real time behaviour.

// Copy text to the buffer for future printing

int silent = 0;
void PushTxt(char *txt) {
	
	int i, l = strlen(txt);

	if (silent) return;

	// If this happens there is a programming bug
 
	if (l > TBLEN) { 		// Can't handle more than TBLEN at a time
		terr = TXT_TOOBIG;	// say error and abort
		return;
	}

	// If the buffer is filling up to fast throw it away and return an error

	if ((l + tsze) >= TBLEN) {	// If there is no room in the buffer
		terr = TXT_OVERFL;	// Buffer overflow
		return;			// Simply stop printing when txt comming too fast
	}

	// Copy the new text onto the ring buffer for later output
	// from the loop idle function

	for (i=0; i<l; i++) {
		txtb[txtw] = txt[i];		// Put char in the buffer and
		txtw = (txtw + 1) % TBLEN;	// get the next write pointer modulo TBLEN
	}
	tsze = (tsze + l) % TBLEN;		// new buffer size
	if (tsze > tmax) tmax = tsze;		// track the max size
}

// Take the next character from the ring buffer and print it, called from the main loop

void PutChar() {
	
	char c[2];			// One character zero terminated string

	if (tsze) {			// If the buffer is not empty

		c[0] = txtb[txtr]; 		// Get the next character from the read pointer
		c[1] = '\0';			// Build a zero terminated string
		txtr = (txtr + 1) % TBLEN; 	// Get the next read pointer modulo TBLEN
		tsze = (tsze - 1) % TBLEN;	// Reduce the buffer size
		Serial.print(c);		// and print the character
	}
}

// Push HTU temperature and humidity from HTU chip

void PushHtu(int flg) {	// If flg is true always push

	double temph = 0.0; 
	double humid = 0.0;

	if ((flg) || ((htu_ok) && ((ppcnt % humtmp_display_rate) == 0))) {
		temph = HtuReadTemperature();
		humid = HtuReadHumidity();
		if (output_format) sprintf(txt,"{'HTU':{'Tmh':%5.3f,'Hum':%4.1f}}\n",temph,humid);
		else sprintf(txt,"%s,HTU,%f,%f\n",CSVERS,temph,humid);
		PushTxt(txt);
	}
}

// Push BMP temperature and altitude from BMP chip

void PushBmp(int flg) {	// If flg is true always push

	float altib = 0.0;
	float tempb = 0.0;
	float presr = 0.0;

	if ((flg) || ((bmp_ok) && ((ppcnt % alttmp_display_rate) == 0))) {
		if (bmp_id == BMP_ON_MB) {
			presr = ps.readPressureMillibars();
			altib = ps.pressureToAltitudeMeters(presr);
			tempb = ps.readTemperatureC();
		}
		if (bmp_id == BMP_ADAFRUIT) {
			tempb = BmpCalcTemp();
			presr = BmpCalcPres();
			altib = BmpCalcAlti();
		}
		if (output_format) sprintf(txt,"{'BMP':{'Tmb':%5.3f,'Prs':%5.3f,'Alb':%4.1f}}\n",tempb,presr,altib);
		else sprintf(txt,"%s,BMP,%f,%f,%f\n",CSVERS,tempb,presr,altib);
		PushTxt(txt);
	}
}

// When the detector is shaken this outputs the (vcn) vibration count the (vax) axis bits
// the latched acceleration values for all 3 axis, the time it happened and the field
// strengths in all three axis. From all this information the Python monitor will be able
// to work out whats going on, sustained vibration or whatever. 

uint32_t old_icount = 0;

void PushVib() { // Push an event when shake detected => Earth Quake 

	if (accl_flag) {
		if (accl_icount != old_icount) {
			old_icount = accl_icount;
			PushTim(1);		// Push these first, and then vib
			PushAcl(1);		// This is the real latched value
			if (output_format) sprintf(txt,"{'VIB':{'Vax':%d,'Vcn':%d}}\n",accl_flag,accl_icount);
			else sprintf(txt,"%s,%d,%d\n",CSVERS,accl_flag,accl_icount);
			PushTxt(txt);
		}
	}
}

// Push the magnetic field strengths in all three axis in micro tesla (gauss)

void PushMag(int flg) {	// Push the mago stuff
	
	if ((flg) || ((acl_id) && ((ppcnt % magnot_display_rate) == 0))) {
		if (output_format) sprintf(txt,"{'MAG':{'Mgx':%f,'Mgy':%f,'Mgz':%f}}\n",mag_fx,mag_fy,mag_fz);
		else sprintf(txt,"%s,MAG,%f,%f,%f\n",CSVERS,mag_fx,mag_fy,mag_fz);
		PushTxt(txt);
	}
}

// Push magnetic event

void PushMev() {

	if (output_format) sprintf(txt,"{'MEV':{'Mev':%d,'Met':%f,'Mdx':%f,'Mdy':%f,'Mdz':%f}}\n",
				   mev_flg,MagToGauss(magnat_event_threshold),MagToGauss(dmag_x),MagToGauss(dmag_y),MagToGauss(dmag_z));
	else sprintf(txt,"%s,MEV,%d,%f,%f,%f,%f\n",
				   CSVERS,mev_flg,MagToGauss(magnat_event_threshold),MagToGauss(dmag_x),MagToGauss(dmag_y),MagToGauss(dmag_z));
	PushTxt(txt);
}

// Push the acceleration values in xyz in meters per sec squared

void PushAcl(int flg) { // Push the accelerometer and compass stuff

	if ((flg) || ((acl_id) && ((ppcnt % accelr_display_rate) == 0))) {
		AclReadData();
		if (output_format) sprintf(txt,"{'ACL':{'Acx':%f,'Acy':%f,'Acz':%f}}\n",acl_fx, acl_fy, acl_fz);
		else sprintf(txt,"%s,%f,%f,%f\n",CSVERS,acl_fx, acl_fy, acl_fz);
		PushTxt(txt);
	}
}

// Push location latitude longitude in degrees so that google maps gets it correct

void PushLoc(int flg) {
		
	if ((flg) || ((ppcnt % latlon_display_rate) == 0)) {
		if (output_format) sprintf(txt,"{'LOC':{'Lat':%f,'Lon':%f,'Alt':%f}}\n",latitude,longitude,altitude);
		else sprintf(txt,"%s,LOC,%f,%f,%f\n",CSVERS,latitude,longitude,altitude);
		PushTxt(txt);
	}
}

// Push timing

void PushTim(int flg) {

	if ((flg) || ((ppcnt % frqutc_display_rate) == 0)) {
		if (output_format) sprintf(txt,"{'TIM':{'Upt':%4d,'Frq':%7d,'Sec':'%s'}}\n",ppcnt,rega0,rdtm);
		else sprintf(txt,"%s,TIM,%d,%d,%6s\n",CSVERS,ppcnt,rega0,rdtm);
		PushTxt(txt);
	}			
}

// Push date from GPS (The DAT field is already used in Python)

void PushDtg(int flg) {

	if ((date_ok) && ((flg) || ((ppcnt % frqdtg_display_rate) == 0))) {
		if (output_format) sprintf(txt,"{'DTG':{'Yer':%4d,'Mnt':%2d,'Day':%1d}}\n",year,month,day);
		else sprintf(txt,"%s,DTG,%4d,%2d,%2d\n",CSVERS,year,month,day);
		PushTxt(txt);
	}			
}

// Push status

void PushSts(int flg, int qsize, int missed) {
uint8_t res;

	if ((flg) || ((ppcnt % status_display_rate) == 0)) {
		if (output_format) {
			sprintf(txt,"{'STS':{'Qsz':%2d,'Mis':%2d,'Ter':%d,'Tmx':%d,'Htu':%d,'Bmp':%d,'Acl':%d,'Mag':%d,'Gps':%d,'Adn':%d,'Gri':%d,'Eqt':%d,'Chm':%d}}\n",
				qsize,missed,terr,tmax,htu_ok,bmp_ok,acl_id,acl_id,gps_ok,adc_samples_per_evt,gps_read_inc,events_display_size,channel_mask);
		} else {
			sprintf(txt,
				"%s,STS,%2d,%2d,%2d,%4d,"
				"%1d,%1d,%1d,%1d,%1d,"
				"%2d,%2d,%2d,%d\n",
				CSVERS,qsize,missed,terr,tmax,
				htu_ok,bmp_ok,acl_id,acl_id,gps_ok,
				adc_samples_per_evt,gps_read_inc,events_display_size,channel_mask);
		}
		PushTxt(txt);
		terr = 0;
	}
}

// Push event queue

void PushEvq(int flg, int *qsize, int *missed) {
		
	struct EventBuf eb;		// Temporary event buffer
	double evtm = 0.0;		// Time since last event or PPS in seconds (< 1.0)
	char stx[16];			// Second text
	int i,j;

	// If there are any events waiting in the event read buffer, put them on the queue

	for (i=0; i<ridx; i++) {
		strncpy(eb.DateTime,rdtm,DATE_TIME_LEN);// Last seconds date/time string 
		eb.Frequency = rega0;			// Ticks between successive PPS pulses
		eb.Ticks = rbuf[i].Tks;			// Ticks since LAST interrupt! (PPS or Event)
		eb.Count = i+1;				// Event index 1..PPS_EVENTS in the second
		for (j=0; j<adc_samples_per_evt; j++) {	// Copy accross ADC values
			eb.Ch0[j] = rbuf[i].Ch0[j];
			eb.Ch1[j] = rbuf[i].Ch1[j];
		}
		*missed = PutQueue(&eb);		// Put buffer on Q, and get missed event count
	}
	*qsize = SzeQueue();	

	if (*qsize >= events_display_size) {

		PushTxt("\n");
		while (PopQueue(&eb)) {	// While ther are events on the queue

			// Calculate the time in seconds of this event in the second
			// N.B. Ticks are since the last event, or from PPS

			if (eb.Count == 1) evtm = 0.0;				// Start a new second
			evtm += ((double) eb.Ticks / (double) eb.Frequency);	// Add time since last event
			sprintf(stx,"%9.7f",evtm);				// It will be 0.something

			// Build string and push it out to the print buffer
			
			adch0[0] = '\0';
			adch1[0] = '\0';

			for (i=0; i<adc_samples_per_evt; i++) {
				if (output_format) {
					sprintf(&adch0[strlen(adch0)],"%d,",eb.Ch0[i]);
					sprintf(&adch1[strlen(adch1)],"%d,",eb.Ch1[i]);
				} else {
					sprintf(&adch0[strlen(adch0)],"%02X",eb.Ch0[i]);
					sprintf(&adch1[strlen(adch1)],"%02X",eb.Ch1[i]);
				}
			}

			adch0[strlen(adch0) -1] = '\0';
			adch1[strlen(adch1) -1] = '\0';

			if ((channel_mask & 0x01) == 0) sprintf(adch0,"0");
			if ((channel_mask & 0x02) == 0) sprintf(adch1,"0"); 
 
			if (output_format) sprintf(txt,
						"{'EVT':{'Evt':%1d,'Frq':%8d,'Tks':%8d,'Etm':%s%s,'Adc':[[%s],[%s]]}}\n",
						eb.Count, eb.Frequency, eb.Ticks, eb.DateTime, index(stx,'.'),adch0,adch1);
			else sprintf(txt,
					"%s,EVT, %7d,%7d,%s%s,%2d,%s,%s\n",
					CSVERS,  eb.Frequency,eb.Ticks,eb.DateTime, index(stx,'.'),adc_samples_per_evt,adch0,adch1);
			PushTxt(txt);
		}
		PushTxt("\n");
	}
}

// Read one input character, we have exactly the same problem with
// the serial line read as with writing, so we need the same work around

void ReadOneChar() {
	char c;

	// Suck in all the characters available on the input stream
	// put as many as will fit in the cmd buffer, and say ready

	if ((irdy == 0) && (Serial.available())) {	// If buffer free
		c = (char) Serial.read();		// Read one char
		if (c == '\n') istp = 1;		// Stop on '\n'
		if ((!istp) && (irdp < (CMDLEN -1))) {
			cmd[irdp] = c;
			irdp = irdp + 1;
			cmd[irdp] = 0;
		}
	} else	irdy = 1;
}

// Implement the command callback functions

void noop(int arg) { };	// That was easy

void vers(int arg) {
	sprintf(cmd_mesg,"VER:%s",FWVERS);
	if (output_format) 
		sprintf(txt,"{'VER':{'Ver':'%s'}}\n",FWVERS);
	else
		sprintf(txt,"%s,VER,%s\n",CSVERS,FWVERS);
	PushTxt(txt);
}

void help(int arg) {	// Display the help
	int i;
	CmdStruct *cms;
	
	sprintf(cmd_mesg,"");
	for (i=0; i<CMDS-1; i++) {
		cms = &cmd_table[i];
		strcat(cmd_mesg,cms->Name);
		strcat(cmd_mesg," ");

		sprintf(txt,"{'HLP':{'Idn':%d,'Nme':'%s','Hlp':'%s'}}\n",cms->Id,cms->Name,cms->Help);
		PushTxt(txt);
	}
}

void htud(int arg) { 
	humtmp_display_rate = arg;
	sprintf(cmd_mesg,"HTU display rate:%d",humtmp_display_rate); 
}

void bmpd(int arg) { 
	alttmp_display_rate = arg; 
	sprintf(cmd_mesg,"BMP display rate:%d",alttmp_display_rate);
}

void locd(int arg) { 
	latlon_display_rate = arg; 
	sprintf(cmd_mesg,"LAT/LON display rate:%d",latlon_display_rate);
}

void timd(int arg) { 
	frqutc_display_rate = arg; 
	sprintf(cmd_mesg,"TIM display rate:%d",frqutc_display_rate);
}

void dtgd(int arg) { 
	frqdtg_display_rate = arg; 
	sprintf(cmd_mesg,"DTG Date GPS display rate:%d",frqdtg_display_rate);
}

void stsd(int arg) { 
	status_display_rate = arg; 
	sprintf(cmd_mesg,"STS display rate:%d",status_display_rate);
}

void evqt(int arg) { 
	events_display_size = arg % (EVENT_QSIZE + 1); 
	sprintf(cmd_mesg,"EVT threshold:%d",events_display_size);
}

void acld(int arg) { 
	accelr_display_rate = arg; 
	sprintf(cmd_mesg,"ACL display rate:%d",accelr_display_rate);
}

void magd(int arg) { 
	magnot_display_rate = arg; 
	sprintf(cmd_mesg,"MAG display rate:%d",magnot_display_rate);
}

void json(int arg) {
	if (arg) {
		output_format = 1;	// JSON output format
		sprintf(cmd_mesg,"Output format set to JSON");
	} else {
		output_format = 0;	// CSV output format
		sprintf(cmd_mesg,"Output format set to CSV");
	}
}
	
void aclt(int arg) { 
	uint8_t val = 0;
	float gthresh;
	accelr_event_threshold = arg & 0x7F; 
	gthresh = 2.0 / (float) 0x7F;
	val = accelr_event_threshold;
	sprintf(cmd_mesg,"ACL sensitivity threshold:%d %fg ",accelr_event_threshold,gthresh);
	BusWrite(acl_ad, 0x32, val, acl_bus);
}

void magt(int arg) {
	uint8_t val;
	float mag_ft;
	if (acl_id == ACL_ON_MB) {
		magnat_event_threshold = 0x7FFF & arg;
		val = magnat_event_threshold >> 8;	// High
		// BusWrite(acl_ad, 0x15, val, 1);		// INT_THS_H_M
		val = magnat_event_threshold & 0xFF;	// Low
		// BusWrite(acl_ad, 0x14, val, 1);		// INT_THS_L_M
		mag_ft = (float) (4.0 * (float) magnat_event_threshold) / (float) 0x7FFF;
		sprintf(cmd_mesg,"MAG sensitivity thresfold:%d = %3.5f Gauss",magnat_event_threshold,mag_ft);
		return;
	}
	sprintf(cmd_mesg,"Feature not available, wrong chip");
	cmd_result = CMD_ERROR;
}

void mpol(int arg) {
	mag_poll_rate = arg;
	if (!mag_poll_rate) {
		sprintf(cmd_mesg,"MAG polling is OFF");
		return;
	}
	sprintf(cmd_mesg,"MAG polling rate once each: %d seconds",mag_poll_rate);
}

// The minimum gps read increment is 3 because
// 0 and 1 read ervery time
// but there is one up the spout, so 2 is a waste of time

#define MIN_GPS_READ_INC 3

void gpri(int arg) { 
	if (arg >= MIN_GPS_READ_INC) {
		gps_read_inc = arg;
		sprintf(cmd_mesg,"GPS read increment:%d seconds",gps_read_inc);
	} else { 
		gps_read_inc = 0;
		sprintf(cmd_mesg,"GPS read every second");
	}

	ReadGpsString();	// Empty GPS output buffer 
}

void nadc(int arg) { 
	adc_samples_per_evt = arg % (ADC_BUF_LEN + 1); 
	if (channel_mask == 3) {
		adc_samples_per_evt >>= 1;
		sprintf(cmd_mesg,"ADC both channels ON, samples per channel:%d",adc_samples_per_evt);
	} else {
		sprintf(cmd_mesg,"ADC one channel ON, samples per channel:%d",adc_samples_per_evt);
	}
}

void rbrk(int arg) {

	if (arg == 0) {
		htu_ok = 0;
		bmp_ok = 0;
		acl_id = 0;
		digitalWrite(POW_ONE,LOW);	// Power off to breakouts
		digitalWrite(POW_TWO,LOW);
		delay(100);	
		sprintf(cmd_mesg,"BRK power OFF");
	} else {
		digitalWrite(POW_ONE,HIGH);	// Power on
		digitalWrite(POW_TWO,HIGH);	
		delay(100);
		HtuReset();
		GetBmpId();
		GetAclId();
		sprintf(cmd_mesg,"BRK power ON, htu_ok:%d bmp_ok:%d acl_ok:%d mag_ok:%d",
			htu_ok,bmp_ok,acl_id,acl_id);
	}
}

void chns(int arg) {
	
	adc_samples_per_evt = 8;
	if (arg < 4)
		channel_mask = arg;
	else
		channel_mask = 3;	// both
	sprintf(cmd_mesg,"CHN channel mask:%d ADC samples set to default:%d",channel_mask,adc_samples_per_evt);
}

// Push result of the last command

void PushCoCo(int flg) {

	if (flg) {		
		sprintf(txt,"{'CMD':{'Cmd':'%s','Res':%d,'Msg':'%s'}}\n",cmd_name,cmd_result,cmd_mesg);
		PushTxt(txt);
	}
}

// Leds On/Off

void leds(int arg) {

	if (arg) {
		leds_on = 1;
		sprintf(cmd_mesg,"LEDS: set to flash");
	} else {
		leds_on = 0;
		sprintf(cmd_mesg,"LEDS: Are off");
	}

	digitalWrite(EVT_PIN,LOW);
	digitalWrite(PPS_PIN,LOW);
}	
		
// Look up a command in the command table for the given command string
// and call it with its single integer parameter

void ParseCmd() {
	int i, p=0, cl=0;
	char *cp, *ep;
	CmdStruct *cms;

	sprintf(cmd_mesg,"");
	sprintf(cmd_name,"%s",cmd);
	cmd_result = 0;

	for (i=0; i<CMDS-1; i++) {
		cms = &cmd_table[i];
		cl = strlen(cms->Name);
		if (strncmp(cms->Name,cmd,cl) == 0) {
			if ((cms->Par) && (strlen(cmd) > cl)) {
				cp = &cmd[cl];
				p = (int) strtoul(cp,&ep,0);
			}
			sprintf(cmd_name,"%s",cms->Name);
			cms->proc(p);
			PushCoCo(1);
			return;
		}
	}
	cmd_result = NO_SUCH_COMMAND;
	sprintf(cmd_mesg,"%s Err:%d No such command",cmd_name,cmd_result);
	PushCoCo(1);
	return;
}

// This waits for a ready buffer from ReadOneChar. Once ready the buffer is
// locked until its been seen here

void DoCmd() {
	if (irdy) {
		if (irdp) ParseCmd();
		bzero((void *) cmd, CMDLEN);
		irdp = 0; irdy = 0; istp = 0;
	}
}

// Arduino main loop does all the user space work

void loop() {

	int missed, qsize;	// Queue vars

	if (displ) {				// Displ is set in the PPS ISR, we will reset it here
		DoCmd();			// Execute any incomming commands
		PushEvq(0,&qsize,&missed);	// Push any events
		PushHtu(0);			// Push HTU temperature and humidity
		PushBmp(0);			// Push BMP temperature and barrometric altitude
		PushLoc(0);			// Push location latitude and longitude
		PushTim(0);			// Push timing data
		PushDtg(0);			// Push the date
		PushUid(0);			// Push unique ID
		
		MagReadData();			// Read magnetic data
		MagConvData();			// Convert to Gauss
		MagPoll();			// Check for magnetic event
		PushMag(0);			// Push mag data
		PushAcl(0);			// Push accelarometer datai

		PushSts(0,qsize,missed);	// Push status
		GetDateTime();			// Read the next date/time from the GPS chip

		displ = 0;			// Clear flag for next PPS

		if ((!gps_id) && ((ppcnt % 20) == 0)) { // Get firmware version ?
			Serial1.println(FMWVERS);
			if (debug_gps) {
				sprintf(txt,"{'TXT':{'Txt':'Send:%s'}}\n",FMWVERS);
				PushTxt(txt);
			}
		}
		if ((date_ok) && (send_gga)) {
			send_gga = 0;
			Serial1.println(RMCGGA);
			if (debug_gps) {
				sprintf(txt,"{'TXT':{'Txt':'Send:%s'}}\n",RMCGGA);
				PushTxt(txt);
			}
		}
		if ((gps_ok) && (!date_ok)) {
			send_gga = 1;
			Serial1.println(RMCZDA);
			if (debug_gps) {
				sprintf(txt,"{'TXT':{'Txt':'Send:%s'}}\n",RMCZDA);
				PushTxt(txt);
			}
		}

		SetHtValue(0);			// Temperature compensated HT setting
		digitalWrite(EVT_PIN,LOW);
	}

	PutChar();	// Print one character per loop !!!
	ReadOneChar();	// Get next input command char
}

// Production tests suit commands and self test features
// and the HT auto setting for the SiPMs
// =====================================================

// Error Codes are 3 digits "Tnn" as follows
// <Test suit number T>,<Error number nn>
// See the PTS Doc

#define NO_HTU 100
#define AMP2A_RANGE 101
#define AMP2B_RANGE 102
#define AMP2A_NO_SIGNAL_BLUE 103
#define AMP2B_NO_SIGNAL_BLUE 104
#define AMP2A_NO_SIGNAL_RED 105
#define AMP2B_NO_SIGNAL_RED 106
#define NO_THRESHOLD 107
#define ASSERTION_FAIL 108

// Test numbers are 3 digits "TSP" as follows  
// <Test suit number T>,<Test in suit S>,<Test sub part P>
// See the PTS Doc

#define TEST_ADC_OFFSETS 110
#define TEST_SIPMS_BLUE 120
#define TEST_SIPMS_RED 121
#define TEST_THRESHOLD 130
#define TEST_HTU 140

#define ADC_BLUE_MIN 0
#define ADC_BLUE_MAX 0xFFFF
#define ADC_RED_MIN 0
#define ADC_RED_MAX 0xFFFF
#define ADC_MIN_OFFSET 2800
#define ADC_MAX_OFFSET 3350

#define CHUNK 32
#define CHUNKS 32
#define PTS_CHBUF_LEN (CHUNKS*CHUNK)
uint16_t ch0[PTS_CHBUF_LEN], ch1[PTS_CHBUF_LEN];

#define LOG_ENTRY 8
#define PTS_LOG (CHUNK*LOG_ENTRY)
char pts_log[PTS_LOG];

void ClearAdcBuf() {
	int i;

	for (i=0; i<PTS_CHBUF_LEN; i++) {
		ch0[i] = 0;
		ch1[i] = 0;
	}
} 
		
uint8_t ReadAdcBuf(int pnts) {
	int i;

	for (i=0; i<pnts; i++) {
		if (channel_mask & 0x01) {
			while((ADC->ADC_ISR & 0x01)==0);	// Wait for channel 0 (2.5us)
			ch0[i] = (uint16_t) ADC->ADC_CDR[0];	// Read ch 0
		}
		if (channel_mask & 0x02) {
			while((ADC->ADC_ISR & 0x02)==0);	// Wait for channel 1 (2.5us)
			ch1[i] = (uint16_t) ADC->ADC_CDR[1];	// Read ch 1
		}
	}
}

// Moving Average

int AveragePoints(uint16_t *fp, int pnts) {
	int i, average;
	uint16_t pnt;

	average = 0;
	for (i=0; i<pnts; i++) {
		pnt = fp[i];
		average += pnt;
	}
	average = average/pnts;
	return average;
}

// Log points

void LogPoints(uint16_t *fp, int pnts) {
	int i;
	uint16_t pnt;
	char *cp;

	sprintf(pts_log,"[");	
	cp = &pts_log[strlen(pts_log)];
	for (i=0; i<pnts; i++) {
		pnt = fp[i];
		sprintf(cp," %d",pnt);
		cp = &pts_log[strlen(pts_log)];
	}
	sprintf(cp," ]");
}

// VoltsPoint

float VoltsPoint(uint16_t pnt) {
	float volts;

	volts = pnt * 3.3 / 4095.0;
	return volts;
}

// Process the ADC values

int CheckPoints(uint16_t *fp, int pnts, int min, int max) {
	int i;
	uint16_t pnt;
	
	for (i=0; i<pnts; i++) {
		pnt = fp[i];
		if ((pnt<=min) || (pnt>=max)) return i+1;
	}
	return 0;
}

// Helper for abts commands

void abts_helper(int arg, int min, int max, int er0, int er1) {
	int av0 = 0;
	int av1 = 0;
	float vl0, vl1;

	av0 = AveragePoints(ch0,CHUNK);
	vl0 = VoltsPoint(av0);
	if (CheckPoints(ch0,CHUNK,min,max)) cmd_result = er0;

	LogPoints(ch0,CHUNK);
	sprintf(txt,"\nCH0:%s Err:%d\n",pts_log,cmd_result);
	PushTxt(txt);

	av1 = AveragePoints(ch1,CHUNK);
	vl0 = VoltsPoint(av1);
	if (CheckPoints(ch1,CHUNK,min,max)) cmd_result = er1;

	LogPoints(ch1,CHUNK);
	sprintf(txt,"\nCH1:%s Err:%d\n",pts_log,cmd_result);
	PushTxt(txt);

	sprintf(cmd_mesg,"ADC: Tst:%d Err:%d Avr CH0:%d %3.2f Vlt Avr CH1:%d %3.2fVlt Smp:%d",
		arg,cmd_result,av0,vl0,av1,vl1,CHUNK);
}

float get_peak_freq(int threshold);
void clear_peaks();

// Test the Analogue board

void abts(int arg) {

	int av0, av1, threshold;
	float vl0, vl1, freq;
	double temph = 0.0, humid = 0.0;

	if (arg == TEST_HTU) {
		if (!htu_ok) {
			cmd_result = NO_HTU;
			sprintf(cmd_mesg,"HTU: Err:%d No breakout available",cmd_result);
			return;
		}

		temph = HtuReadTemperature();
		humid = HtuReadHumidity();
		sprintf(cmd_mesg,"HTU: Err:%d :Temp:%5.3f Hum:%4.1f",cmd_result,temph,humid);
		return;
	}
		
	if (arg == TEST_ADC_OFFSETS) {
		ClearAdcBuf();
		ReadAdcBuf(CHUNK);
		abts_helper(arg,ADC_MIN_OFFSET,ADC_MAX_OFFSET,AMP2A_RANGE,AMP2B_RANGE);
		return;
	}

	if (arg == TEST_SIPMS_BLUE) {
		digitalWrite(BLUE_LED_PIN,HIGH);
		ClearAdcBuf();
 		ReadAdcBuf(CHUNK);
		digitalWrite(BLUE_LED_PIN,LOW);
		abts_helper(arg,ADC_BLUE_MIN,ADC_BLUE_MAX,AMP2A_NO_SIGNAL_BLUE,AMP2B_NO_SIGNAL_BLUE);
		return;
	}

	if (arg == TEST_SIPMS_RED) {
		digitalWrite(RED_LED_PIN,HIGH);
 		ClearAdcBuf();
		ReadAdcBuf(CHUNK);
		digitalWrite(RED_LED_PIN,LOW);
		abts_helper(arg,ADC_RED_MIN,ADC_RED_MAX,AMP2A_NO_SIGNAL_RED,AMP2B_NO_SIGNAL_RED);
		return;
	}

	if (arg == TEST_THRESHOLD) {
		
		// The threshold is above the background, so the test range 100..2000 seems reasonable

		for (threshold=100; threshold<=2000; threshold+=100) {
			clear_peaks();
			freq = get_peak_freq(threshold);
			sprintf(txt,"\nADC: Tst%d Threshold:%d Freq:%3.2f\n",arg,threshold,freq);
			PushTxt(txt);
			if (freq <= 10.0) {
				sprintf(cmd_mesg,"ADC: Tst:%d PASS Threshold:%d Freq:%32.f",arg,threshold,freq);
				return;
			}
		}
		cmd_result = NO_THRESHOLD;
		sprintf(cmd_mesg,"ADC: Tst:%d FAILED, no threshold could be found");
		return;
	}

	sprintf(cmd_mesg,"Illegal test number:%d",arg);
	
	cmd_result = ASSERTION_FAIL;
	return;
}

// Peaks corresponding to events

struct Peak {
	int Start;
	int Width;
	int Loops;
};
	
#define PEAKS 1000
static struct Peak peaks[PEAKS];	// Event ticks buffer
int peak_index = 0;			// Points to free peak in buffer

void clear_peaks() {
	int i;
	struct Peak *pp;	// Points to current peak
	
	peak_index = 0;

	for (i=0; i<PEAKS; i++) {
		pp = &peaks[i];
		pp->Start = 0;
		pp->Width = 0;
		pp->Loops = 0;
	}
}

// Find the threshold for event rate lower than 10Hz
// Frequency is returned for given threshold

float get_peak_freq(int threshold) {	// Threshold to test

	int i;
	int av0,av1;		// Background value is the running average

	int start = 0;		// Start of event index
	int width = 0;		// Width of the event
	int loops = 0;		// Loops correspond to time

	struct Peak *pp;	// Points to current peak

	unsigned long ewid = 0;	// Sum of times between events
	int ectm = 0;		// Event current time
	int estr = 0;		// First event start time in a pair


	float freq;		// Peak occurence frequency

	ClearAdcBuf();
	ReadAdcBuf(PTS_CHBUF_LEN); // Read 4096 point chunk
		
	// Calculate the background levels

	av0 = AveragePoints(ch0,PTS_CHBUF_LEN);
	av1 = AveragePoints(ch1,PTS_CHBUF_LEN);

	// Try to find the next 1000=PEAKS peaks

	while (true) {	// Collect PEAKS peaks
		
		ClearAdcBuf();
		ReadAdcBuf(PTS_CHBUF_LEN); // Read 4096 point chunk
		
		// Look for simultaneous points above the background and save them

		for (i=0; i<PTS_CHBUF_LEN; i++) {
			if ((ch0[i] > av0 + threshold) 
			&&  (ch1[i] > av1 + threshold)) {
				if (!start) {
					start = i+1;
					if (peak_index < (PEAKS -1)) {
						pp = &peaks[peak_index++];
						pp->Start = start;
						pp->Loops = loops;
					} else
						break;
				}
				width++;
			 } else {
				if (start) {
					pp->Width = width;

					sprintf(txt,"Peak:%d S:%d W:%d L:%d\n",
						peak_index,pp->Start,pp->Width,pp->Loops);
					PushTxt(txt);
 
					start = 0;
					width = 0;
				}
			}

			PutChar();
		}
		
		if (++loops > 1000) break;
	}

	// Calculate the average time between peaks
	
	estr = 0;
	for (i=peak_index; i>=0; i--) {
		pp = &peaks[i];
		ectm = (pp->Loops * PTS_CHBUF_LEN) + pp->Start -1;	// Corresponds to time
		if (estr == 0) estr = ectm;
		ewid += abs(estr - ectm);
		estr = ectm;
	}
	
	if (ewid < 1) 
		freq = 1000.0;
	else
		freq = ((float) peak_index * 1385000.0) / (float) ewid;

	sprintf(txt,"Peaks:%d Sum:%d Freq:%3.2f\n",peak_index,ewid,freq);
	PushTxt(txt);

	return freq;
}

// =============================================
// Test for GPS reciever
// Basically if the GPS ID is OK it means the chip can be configured
// and read back. Then if the PPS arrives its locked and it arrives.
// The global gps_ok is set in the ISR when a PPS arrives, else the PLL takes over
// The global gps_id is set when the GPS NMEA string is read

#define NO_GPS_FOUND 503
#define NO_PPS 504

void dgps(int arg) {
	debug_gps = arg; // Controls printing GPS NMEA strings for debugging
	sprintf(cmd_mesg,"GPS: Debug:%d",debug_gps);
	Serial1.println(FMWVERS);
	sprintf(txt,"{'TXT':{'Txt':'Send:%s'}}\n",FMWVERS);
	PushTxt(txt);
}

void gpid(int arg) {
	cmd_result = 0;
	Serial1.println(FMWVERS);

	if (gps_id == ADAFRUIT_GPS) {
		sprintf(cmd_mesg,"GPS: PASS Id:%d Adafruit Ultimate GPS found",gps_id);
		return;
	}
	if (gps_id == QUECTEL76_GPS) {
		sprintf(cmd_mesg,"GPS: PASS Id:%d QUECTL L76 chip GPS found",gps_id);
		return;
	}
	if (!gps_ok) {
		sprintf(cmd_mesg,"GPS: FAIL No GPS found YET");
		cmd_result = NO_GPS_FOUND;
		return;
	}
	sprintf(cmd_mesg,"GPS: PASS GPS found, but no ID available");
	cmd_result = NO_GPS_FOUND;
	return;
}

void gpps(int arg) {
	cmd_result = 0;

	if (gps_ok)
		sprintf(cmd_mesg,"GPS: PASS PPS Arriving");
	else {
		sprintf(cmd_mesg,"GPS: FAIL No PPS detected YET");
		cmd_result = NO_PPS;
	} 
	return;
}

// =============================================
// Test for accelerometer

#define TEST_ACL_INTERRUPT 730
#define NO_ACL_FOUND 700

void acts(int arg) {

	if (acl_id == ACL_ADAFRUIT) {
		sprintf(cmd_mesg,"ACL: PASS Bus:%d Adr:0x%02X Adafruite breakout",acl_bus,ACL_BUS_0_ADDR);
		return;
	}
	if (acl_id == ACL_ON_MB) {
		sprintf(cmd_mesg,"ACL: PASS Bus:%d Adr:0x%02X LMS303 on MB",acl_bus,ACL_BUS_1_ADDR);
		return;
	}
	sprintf(cmd_mesg,"ACL: FAIL No Accelerometer found");
	cmd_result = NO_ACL_FOUND;
	return;
}

// ==========================================
// I2C Bus scan

void i2cs(int arg) {
	
	int adr, err, cnt;

	if ((arg < 0) || (arg > 1)) {
		sprintf(cmd_mesg,"No such bus:%d [0-1]",arg);
		cmd_result = ASSERTION_FAIL;
		return;
	}
	sprintf(cmd_mesg,"Bus:%d ",arg);

	cnt = 0;	
	for (adr=1; adr<127; adr++) {
		if (arg==1) {
    			Wire1.beginTransmission(adr);
    			err = Wire1.endTransmission();
		} else {
	    		Wire.beginTransmission(adr);
    			err = Wire.endTransmission();
		}
		if (!err) {
			sprintf(txt,"0x%02X ",adr);
			strcat(cmd_mesg,txt);
			cnt++;
		}
	}
	sprintf(txt,"Found:%d Devices",cnt);
	strcat(cmd_mesg,txt);
}

// =============================================
// Dump LSM303

void d303(int arg) {
	int i;
	uint8_t val, reg;

	if ((arg<1) || (arg>10))
		arg = 1;

	sprintf(txt,"{'TXT':{'Txt':'LM303 ");
	PushTxt(txt);
	for (i=0; i<arg; i++) {
                for (reg=0x00; reg<=0x3F; reg++) {
			val = BusRead(acl_ad, reg, acl_bus);
                        sprintf(txt,"%02X/%02X ",reg,val);
                        PushTxt(txt);
                }
		sprintf(txt,"'}}\n",reg,val);
		PushTxt(txt);
	
		if (acl_id == ACL_ADAFRUIT) {
	                sprintf(txt,"\nAdaFruit:MagRegs: ");
        	        PushTxt(txt);
                	for (reg=0x00; reg<=0x32; reg++) {

	                        if ((reg==0x3E) || (reg==0x3F)) continue;
        	                if ((reg>=0x0D) && (reg<=0x30)) continue;
				val = BusRead(mag_ad, reg, acl_bus);
                        	sprintf(txt,"%02X:%02X ",reg,val);
				PushTxt(txt);
        	        }
		}
		sprintf(txt,"\nLoop:%02d\n",i+1);
		PushTxt(txt);
	}
}

// =============================================
// Dump HTU registers

#define HTUREGS 7
typedef enum {			TTmpH,   THumH,   TTmpNH,   THumNH,   WUsr,   RUsr,   SRes   };
uint8_t hturegs[HTUREGS] = {	0xE3,    0xE5,    0xF3,     0xF5,     0xE6,   0xE7,   0xFE   };

#define HTU_TIMEOUT 50

static uint8_t htu_rd=0;
void HtuReset() {

	Wire1.beginTransmission(HTU_BUS_1_ADDR);
	Wire1.write(hturegs[SRes]);
	PushBusError(Wire1.endTransmission(),HTU_BUS_1_ADDR,hturegs[SRes],1);
	bus_err = Wire1.endTransmission();
	delay(15);

	Wire1.beginTransmission(HTU_BUS_1_ADDR);
	Wire1.write(hturegs[RUsr]);
	bus_err = Wire1.endTransmission();
	PushBusError(Wire1.endTransmission(),HTU_BUS_1_ADDR,hturegs[RUsr],1);
	Wire1.requestFrom(HTU_BUS_1_ADDR, 1);
	htu_rd = Wire1.read();
	htu_ok = htu_rd;
}

static uint8_t htu_th=0, htu_tl=0, htu_tc=0, htu_tmo=0; 
void HtuTemp() {

	htu_tmo = 0;
	Wire1.beginTransmission(HTU_BUS_1_ADDR);
	Wire1.write(hturegs[TTmpH]);
	PushBusError(Wire1.endTransmission(),HTU_BUS_1_ADDR,hturegs[TTmpH],1);
	delay(50);

	Wire1.requestFrom(HTU_BUS_1_ADDR, 3);
	while (!Wire1.available()) {
		if (htu_tmo++ > HTU_TIMEOUT) 
			break;
		else 
			delay(1);
	}
	htu_th = Wire1.read();
	htu_tl = Wire1.read();
	htu_tc = Wire1.read();
}

static uint8_t htu_hh=0, htu_hl=0, htu_hc=0, htu_hmo;
void HtuHumid() {

	htu_hmo = 0;
	Wire1.beginTransmission(HTU_BUS_1_ADDR);
	Wire1.write(hturegs[THumH]);
	PushBusError(Wire1.endTransmission(),HTU_BUS_1_ADDR,hturegs[THumH],1);
	bus_err = Wire1.endTransmission();
	delay(50);

	Wire1.requestFrom(HTU_BUS_1_ADDR, 3);
	while (!Wire1.available()) {
		if (htu_hmo++ > HTU_TIMEOUT) 
			break;
		else 
			delay(1);
	}

	htu_hh = Wire1.read();
	htu_hl = Wire1.read();
	htu_hc = Wire1.read();
}

float HtuConvTemp() {
	float temp;
	short Stemp;

	Stemp = (htu_th << 8) | htu_tl;

	// See data sheet for this formula

	temp = -46.85 + (175.72 * Stemp / ((float) (unsigned int) 0x10000)); 
	return temp;
}

float HtuConvHumid() {
	float rh;
	short Srh;

	Srh = (htu_hh << 8) | htu_hl;

	// See data sheet for this formula

	rh = -6.0 + (125.0 * Srh / ((float) (unsigned int) 0x10000));
	return rh;
}

float HtuReadTemperature() {
	if (!htu_rd) HtuReset();
	HtuTemp();
	return HtuConvTemp();
}

float HtuReadHumidity() {
	if (!htu_rd) HtuReset();
	HtuHumid();
	return HtuConvHumid();
}

#define NO_HTU 800

void dhtu(int arg) {
	int i, j;
	uint8_t val, reg;
	char *cp;

	if ((arg<1) || (arg>10))
		arg = 1;

	if (!htu_rd) HtuReset();
	if ((htu_rd==0) || (htu_rd==0xFF)) {
		htu_rd = 0; 
		htu_ok = 0; 
		sprintf(cmd_mesg,"HTU: FAIL no chip found");
		cmd_result = NO_HTU;
	}

	for (i=0; i<arg; i++) {
		sprintf(txt,"\nHtu");
		PushTxt(txt);

		HtuTemp();
		HtuHumid();

                sprintf(txt,"rd:0x%02X th:0x%02X tl:0x%02X tc:0x%02X hh:0x%02X hl:0x%02X hc:0x%02X t:%3.5fc rh:%3.5f",
			htu_rd, htu_th, htu_tl, htu_tc, htu_hh, htu_hl, htu_hc,
			HtuConvTemp(),HtuConvHumid());
                PushTxt(txt);
		sprintf(cmd_mesg,"%s",txt);

		sprintf(txt,"\nLoop:%02d\n",i+1);
		PushTxt(txt);
	}
}

// =============================================
// Barometric pressure chips

#define BMP_BUS_1_ADDR 0x5D	// LPS25H chip on bus 1
#define BMP_BUS_0_ADDR 0x77	// BMP085 chip on bus 0
#define LPS25H_WHO_AM_I 0x0F
#define LPS25H_ID 0xBD
#define BMP085_WHO_AM_I 0xD0
#define BMP805_ID 0x55

#define NO_BMP 900

void GetBmpId() {
	uint8_t id;

	if (bmp_id) {
		bmp_ok = bmp_id;
		return;
	}

	bmp_bus = 1;
	id = BusRead(BMP_BUS_1_ADDR,LPS25H_WHO_AM_I,1);	
	if (id == LPS25H_ID) { 
		bmp_id = BMP_ON_MB;
		bmp_ad = BMP_BUS_1_ADDR;
		bmp_ok = bmp_id;
		return;
	}

	bmp_bus = 0;
        id = BusRead(BMP_BUS_0_ADDR, BMP085_WHO_AM_I, 0);
        if (id == BMP805_ID) {
		bmp_id = BMP_ADAFRUIT;
		bmp_ad = BMP_BUS_0_ADDR;
		bmp_ok = bmp_id;
		return;
	}

	bmp_bus = 0;
	bmp_id = 0;
	bmp_ad = 0;
}

void bmid(int arg) {

	GetBmpId();
	if (bmp_id == BMP_ON_MB) {
		sprintf(cmd_mesg,"BMP: PASS: Found LPS25H on MB bus 1");
		return;
	}

	if (bmp_id == BMP_ADAFRUIT) {
		sprintf(cmd_mesg,"BMP: PASS: Found BMP085 on Adafruit breakout bus 0");
		
		sprintf(txt,"{'TXT':{'Txt':'BMP %s'}}\n",BmpDebug());
		PushTxt(txt);

		return;
	}

	sprintf(cmd_mesg,"BMP: FAIL: No chip answered");
	cmd_result = NO_BMP;
	return;
}

// ==============================================
// Cosmic Pi Power supply open loop controller

int receive_on = 0;

byte bitBang(byte _send)  {

	byte _receive = 0;
	digitalWrite(SS_PIN, LOW);
	
	for(int i=0; i<8; i++) {
		digitalWrite(MOSI_PIN, bitRead(_send, 7-i));
		digitalWrite(SCK_PIN, HIGH);               
		if (receive_on) 
			bitWrite(_receive, i, digitalRead(MISO_PIN));

		digitalWrite(SCK_PIN, LOW);

		if (receive_on) 
			digitalWrite(MOSI_PIN, LOW);
	}

	digitalWrite(SS_PIN, HIGH);

	if (receive_on) 
		return _receive;
	return 0xFF;
}

uint8_t puval = 0;	// 0x6E Seems about right at room temp
void wrpu(int arg) {
	uint8_t send, recv;

	send = (0xFF & arg);
	recv = bitBang(send);
	puval = send;
	
	if (receive_on) 
		sprintf(cmd_mesg,"HV: Send:0x:%02X Recv:0x%02X MAX1923 PU",send,recv);	
	else
		sprintf(cmd_mesg,"HV: Send:0x:%02X MAX1923 PU",send);

	PushHpu();
	return;
}

void rcpu(int arg) {

	if (arg == 1)
		receive_on = 1;	
	else if (arg == 2)
		receive_on = 0;
	
	if (receive_on) 
		sprintf(cmd_mesg,"HV Receiving ON");
	else
		sprintf(cmd_mesg,"HV receivinf OFF");

	return;
}

// =================================================================================================
// Control the thresholds on MAX5387
// This is the HT voltage automatic setting for the SiPMs. An initial guess is made by looking up a
// value in the temperature array. If two many events arrive the voltage is decreased, if no events
// arrive the voltage is increased. There is also a threshold value that is set to be above the 
// the noise value.

#define MAX_ADDR 0x28

#define A_ONLY 0x11
#define B_ONLY 0x12
#define A_AND_B 0x13

uint8_t abreg = 0;	// Auto
uint8_t thval = 0x30;	// Nice initial value
uint8_t athv0 = 0;	// Automatic threshold hardware value channel 0
uint8_t athv1 = 0;

void SetThrsValue() {
	float tvalf;		// Tempory voltage value
	int   tvali;		// Temp threshold hardware value

	if (abreg) return;	// Not in auto

	tvalf = VoltsPoint(avc0);		// Average ADC background value
	tvali = ((tvalf*256.0)/3.3) + thval;	// ADC background + thval
	if (tvali > 0xFF) tvali = 0xFF;		// Clamp at 8 bits
	athv0 = tvali;
	BusWrite(MAX_ADDR,A_ONLY,athv0,0);	// Set threshold on channel 0 bus 0

	tvalf = VoltsPoint(avc1);
	tvali = ((tvalf*256.0)/3.3) + thval;
	if (tvali > 0xFF) tvali = 0xFF;
	athv1 = tvali;
	BusWrite(MAX_ADDR,B_ONLY,athv1,0);	// Set channel 1 threshold

	PushHpu();
}

void wrth(int arg) {
	int err = 0;

	thval = (uint8_t) arg;

	if (abreg == 0) {
		sprintf(cmd_mesg,"MAX Threshold Auto increment set:0x%02X",thval);
		SetThrsValue();
		PushHpu();
		return;
	}

	BusWrite(MAX_ADDR,abreg,thval,0);
	if (bus_err == 0) {
		PushHpu();
		if (abreg == A_AND_B) {
			sprintf(cmd_mesg,"MAX Threshold	A_and_B set: 0x%02X",thval);
			return;
		}
		if (abreg == A_ONLY) {
			sprintf(cmd_mesg,"MAX Threshold A_Only set: 0x%02X",thval);
			return;
		}
		if (abreg == B_ONLY) {
			sprintf(cmd_mesg,"MAX Threshold B_Only set: 0x%02X",thval);
			return;
		}
	}
	sprintf(cmd_mesg,"MAX5387 Device did not answer, err:%d",bus_err);
}

// Control how threshold value is used

void abth(int arg) {
	uint8_t val = 0;
	
	if (arg == 0) val = 0; 
	if (arg == 1) val = A_ONLY;
	if (arg == 2) val = B_ONLY;
	if (arg == 3) val = A_AND_B;
	abreg = val;

	if (abreg)
		sprintf(cmd_mesg,"MAX Threshold hardware select set: 0x%02X",abreg);
	else {
		SetThrsValue();
		sprintf(cmd_mesg,"MAX Threshold set:AUTO");
	}
	PushHpu();
}

// temperature range -10 to +40 initial HT values

static uint8_t ht_vals[51] = {
	// -10  -09  -08  -07  -06  -05  -04  -03  -02  -01
	   0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xEF,0xEF,0xEF,
	// 00   01   02   03   04   05   06   07   08   09
	   0xEF,0xEF,0xEF,0xEF,0xEF,0xDF,0xDF,0xCF,0xCF,0xBF,
	// 10   11   12   13   14   15   16   17   18   19
	   0xBF,0xBF,0xAF,0x9F,0x9F,0x8F,0x8F,0x7F,0x7F,0x7F,
	// 20   21   22   23   24   25   26   27   28   29
	   0x6F,0x6F,0x6E,0x6E,0x6E,0x6E,0x6E,0x6E,0x6E,0x6E,
	// 30   31   32   33   34   35   36   37   38   39
	   0x6D,0x6C,0x6B,0x6A,0x69,0x68,0x67,0x66,0x65,0x64,
	// 40  
	   0x63 };

uint8_t nhtval = 0;	// New HT value
uint8_t ohtval = 0;	// Old HT value
uint8_t incadj = 0;	// Increment adjustment
uint8_t decadj = 0;	// Decrement adjustment

void SetHtValue(int flg) {
	int itmp = 0;
	float htmp = 0.0;

	// If the flag is set switch off the HT, this happens at startup
	// because the reset has no effect on the HT value

	if (flg) {
		nhtval = 0;
		bitBang(nhtval);
		SetThrsValue();
		PushHpu();
		return;
	}

	// If manual control is requested use that value
	// Automatic control happens when the puval is zero

	if (puval) {
		incadj = 0;
		decadj = 0;
		inc_ht_flg = 0;
		dec_ht_flg = 0;
		return;
	} 

	// N.B. The bigger the value, the lower the HT voltage

	// More than 10 extra events in a second, adjust

	if (inc_ht_flg > 10) {	
		if (decadj > 0) decadj--;
		else incadj++;
		inc_ht_flg = 0;
	}
	
	// No events detected for 5 seconds, adjust

	if (dec_ht_flg > 5) {
		if (incadj > 0) incadj--;
		else decadj++;
		dec_ht_flg = 0;
	}

	// Calculate the HT voltage setting

	htmp = HtuReadTemperature();		// Read the temperature
	itmp = (int) round(htmp) + 10;	
	if (itmp <  0) itmp = 0;
	if (itmp > 51) itmp = 51;		// Clamp it 0..51 (-10..40)
	nhtval = ht_vals[itmp] + incadj -decadj;// Look it up
	if (nhtval < 0x40) nhtval = 0x40;	// Going less than 40 could damage the SiPMs

	if (ohtval != nhtval) {			// If its changed write the new value
		ohtval = nhtval;
		bitBang(nhtval);		  // Set HT value
		BusWrite(MAX_ADDR,abreg,thval,0); // Set threshold
		SetThrsValue();
		PushHpu();			  // Send message
	}
}

void PushHpu() {
	if (output_format)
		sprintf(txt,"{'HPU':{'Ato':'0x%02X','Hpu':'0x%02X','Th0':'0x%02X','Th1':'0x%02X','Thr':'0x%02X','Abr':'0x%02X'}}\n",nhtval,puval,athv0,athv1,thval,abreg);
	else
		sprintf(txt,"%s,HPU,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,%d\n",CSVERS,nhtval,puval,athv0,athv1,thval,abreg);
	PushTxt(txt);
}

// ==========================================================================
// Get Arduino DUE unique ID

__attribute__ ((section (".ramfunc")))
void _EEFC_ReadUniqueID( unsigned int * pdwUniqueID ) {
	unsigned int status;

	/* Send the Start Read unique Identifier command (STUI) 
	 * by writing the Flash Command Register with the STUI command.
	 */
	EFC1->EEFC_FCR = (0x5A << 24) | EFC_FCMD_STUI;
	do {
		status = EFC1->EEFC_FSR ;
	} while ((status & EEFC_FSR_FRDY) == EEFC_FSR_FRDY);

	/* The Unique Identifier is located in the first 128 bits of the 
	 * Flash memory mapping. So, at the address 0x400000-0x400003. 
	 */
	pdwUniqueID[0] = *(uint32_t *)IFLASH1_ADDR;
	pdwUniqueID[1] = *(uint32_t *)(IFLASH1_ADDR + 4);
	pdwUniqueID[2] = *(uint32_t *)(IFLASH1_ADDR + 8);
	pdwUniqueID[3] = *(uint32_t *)(IFLASH1_ADDR + 12);

	/* To stop the Unique Identifier mode, the user needs to send the Stop Read unique Identifier
	 * command (SPUI) by writing the Flash Command Register with the SPUI command. 
	 */
	EFC1->EEFC_FCR = (0x5A << 24) | EFC_FCMD_SPUI ;

	/* When the Stop read Unique Unique Identifier command (SPUI) has been performed, the
	 * FRDY bit in the Flash Programming Status Register (EEFC_FSR) rises. 
	 */
	do {
		status = EFC1->EEFC_FSR ;
	} while ((status & EEFC_FSR_FRDY) != EEFC_FSR_FRDY);
}

int uid_ok = 0;
char uidtxt[132];

void GetUid() {
	unsigned int uidata[5];
	if (!uid_ok) {
		_EEFC_ReadUniqueID(uidata);
		sprintf(uidtxt,"%08X%08X%08X%08X",uidata[0],uidata[1],uidata[2],uidata[3]);
		uid_ok = 1;
	}
}

void PushUid(int flg) {
	if ((flg) || ((uid_ok) && ((ppcnt % uid_display_rate) == 0))) {
		if (output_format) sprintf(txt,"{'UID':{'Uid':'%s'}}\n",uidtxt);
		else sprintf(txt,"%s,UID,0x%08X%08X%08X%08X\n",CSVERS,uidtxt);
		PushTxt(txt);
	}
}

void unid(int arg) {
	if (arg) 
		uid_display_rate = arg;
	GetUid();
	PushUid(1);
	sprintf(cmd_mesg,"UNID:%s display_rate:%d",uidtxt,uid_display_rate);
}

void strg(int arg) {
	sprintf(cmd_mesg,"STRG: A:%d B:%d",striga,strigb);
	if (arg) {
		striga = 0;
		strigb = 0;
	}
}
