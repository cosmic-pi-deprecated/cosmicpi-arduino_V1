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

// Julian Lewis lewis.julian@gmail.com

#define VERS "2016/July/27"

// In this sketch I am using an Adafruite ultimate GPS breakout which exposes the PPS output
// The Addafruite Rx is connected to the DUE TX1 (Pin 18) and its Tx to DUE RX1 (Pin 19)
// The Adafruite 3.3V power is provided from the DUE 3.3V and ground pins
// N.B. Go to the Adafruit downloads page and copy Adafruit_GPS.h and Adafruit_GPS.cc to
// your sketch directory. 

// The output from this program is processed by a Python monitor on the other end of the
// serial line. There has to be mutual aggreement between this program and the monitor.

// Output strings
// All fields in all output strings conform to the json standard

// Here is the list of all records where 'f' denotes float and 'i' denotes integer ...
// {'HTU':{'Tmh':f,'Hum':f}}
// HTU21DF record containing Tmh:temperature in C Hum:humidity percent
//
// {'BMP':{'Tmb':f,'Prs':f,'Alb':f}}
// BMP085 record containing Tmb:temperature Prs:pressure Alb:Barrometric altitude
//
// {'VIB':{'Vax':i,'Vcn':i}}
// Vibration record containing Vax:3 bit xyz direction mask Vcn:vibration count
// This record is always immediatly followed by 3 more records, TIM, ACL, and MAG
//
// {'MAG':{'Mgx':f,'Mgy':f,'Mgz':f}}
// LSM303DLH magnatometer record containing Mgx:the x field strength Mgy:the y field Mgz:ther z field
//
// {'MOG':{'Mox':f,'Moy':f,'Moz':f}}
// LSM303DLH magnatometer record containing Mox:x orientation Moy:y orientation Moz:z orientation
// This record is optional, by default its turned off (it can always be calculated later - Python)
//
// {'ACL':{'Acx':f,'Acy':f,'Acz':f}}
// LSM303DLH acclerometer record containing Acx:the x acceleration Acy:the y acceleration Acz:the z acceleration
// If this record immediatly follows a VIB record the fields were hardware latched when the g threshold was exceeded
//
// {'AOL':{'Aox':f,'Aoy':f,'Aoz':f}}
// LSM303DLH accelerometer record containing Aox:x orientation Aoy:y orientation Aoz:z orientation
// This record is optional, by default its turned off (it can always be calculated later - Python)
//
// {'LOC':{'Lat':f,'Lon':f,'Alt':f}}
// GPS location record containing Lat:latitude in degrees Lon:longitude in degrees Alt:altitude in meters
//
// {'TIM':{'Upt':i,'Frq':i,'Sec':i}}
// Time record containing Upt:up time seconds Frq:counter frequency Sec:time string
//
// {'STS':{'Qsz':i,'Mis':i,'Ter':i,'Htu':i,'Bmp':i,'Acl':i,'Mag':i, 'Gps':i}}
// Status record containing Qsz:events on queue Mis:missed events Ter:buffer error 
// Htu:status Bmp:status Acl:status Mag:status Gps:ststus
//
// {'EVT':{'Evt':i,'Frq':i,'Tks':i,'Etm':f,'Adc':[[i,i,i,i,i,i,i,i][i,i,i,i,i,i,i,i]]}}
// Event record containing Evt:event number in second Frq:timer frequency Tks:ticks since last event in second 
// Etm:event time stamp to 100ns Adc:[[Channel 0 values][Channel 1 values]]

// N.B. These records pass the data to a python monitor over the serial line. Python has awsome string handling and looks them up in
// associative arrays to build records of any arbitary format you want. So this is only the start of the story of record processing.
// N.B. Also some of these records are sent out at regular intervals and or when an event occurs.

// This program also accepts commands sent to it on the serial line.
// When a command arrives it is immediatly executed.

#include <time.h>
#include <Wire.h>

#include "Adafruit_BMP085_U.h"	// Barrometric pressure

#include "Adafruit_HTU21DF.h"	// Humidity and temperature sensor

// GPS chips typically return NMEA strings over a serial line.
// They can be programmed to send different NMEA strings according to what you configure.
// The string RMCGGA has the altitude but misses the yy/mm/dd from the date, it only has hh/mm/ss.
// This is easilly made up for in the Python monitor which gets this information from it system time.

#include "Adafruit_GPS.h"	// GPS chip
#define GPSECHO true

#include "Adafruit_L3GD20_U.h"	// Magoscope

// WARNING: I had to modify this library, its no longer standard 
#include "Adafruit_LSM303_U.h"	// Accelerometer and magnentometer/compass

#include "Adafruit_10DOF.h"	// 10DOF breakout driver - scale to SI units

// Configuration constants

// The size of the one second event buffer
#define PPS_EVENTS 8	// The maximum number of events stored per second
#define ADC_BUF_LEN 8  // Number of ADC values per event

// This is the event queue size
#define EVENT_QSIZE 32	// The number of events that can be queued for serial output

// Handle text buffer serial output overflow errors
// When the output buffer overflows due to data comming too fast, we just stop printing due
// to insufficient bandwidth or slow things down if HANDLE_OVERFLOW is set (not recomended)
// #define HANDLE_OVERFLOW

// This is the text ring buffer for real time output to serial line with interrupt on
#define TBLEN 4096	// Serial line output ring buffer size

// Define some output debug pins to monitor whats going on via an oscilloscope
#define PPS_PIN 13	// PPS (Pulse Per Second) and LED
#define EVT_PIN 12	// Cosmic ray event detected
#define FLG_PIN 11	// Debug flag

// Power pins for power on/off the breakouts after a reset
// The DUE makes a reset if the USB connection is restarted
// The AddaFruit breakouts loose it when this happens and the
// only way to recover them is a power cycle

#define POW_ONE 8	// High power 15ma
#define POW_TWO 9	// 15ma

// For siesmic event input
#define ACL_PIN 10	// Accelarometer INT1 interrupt pin

// Baud rates
#define SERIAL_BAUD_RATE 9600	// Serial line 
#define GPS_BAUD_RATE 9600	// GPS and Serial1 line

// Instantiate external hardware breakouts

Adafruit_GPS		gps(&Serial1);			// GPS Serial1 on pins RX1 and TX1
boolean			gps_ok = false;			// Chip OK flag
boolean			time_ok = false;		// Time read from GPS OK

Adafruit_HTU21DF	htu = Adafruit_HTU21DF();	// Humidity and temperature measurment
boolean			htu_ok = false;			// Chip OK

#define BMPID 18001
Adafruit_BMP085_Unified	bmp = Adafruit_BMP085_Unified(BMPID);	// Barometric pressure
boolean			bmp_ok = false;

// The 10DOF isn't a chip, its just a utility to convert say mago values into headings etc

Adafruit_10DOF		dof = Adafruit_10DOF();		// The 10 Degrees-Of-Freedom DOF breakout
boolean			dof_ok = false;			// board driver, scales units to SI

#define ACLID 30301
Adafruit_LSM303_Accel_Unified acl = Adafruit_LSM303_Accel_Unified(ACLID);	// Accelerometer Compass
			boolean acl_ok = false;

#define MAGID 30302
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(MAGID);		// Magoscope
boolean			mag_ok = false;

// Control the output data rates by setting defaults, these values can be modified at run time
// via commands from the serial interface. Some output like position isn't supposed to be changing
// very fast if at all, so no need to clutter up the serial line with it. The Python monitor keeps
// the last sent values when it builds event messages to be sent over the internet to the server
// or logged to a file.

uint32_t latlon_display_rate = 12;	// Display latitude and longitude each X seconds
uint32_t humtmp_display_rate = 12;	// Display humidity and HTU temperature each X seconds
uint32_t alttmp_display_rate = 12;	// Display altitude and BMP temperature each X seconds
uint32_t frqutc_display_rate = 1;	// Display frequency and UTC time each X seconds
uint32_t status_display_rate = 4;	// Display status (UpTime, QueueSize, MissedEvents, HardwareOK)
uint32_t accelr_display_rate = 1;	// Display accelarometer x,y,z
uint32_t magnot_display_rate = 12;	// Display magnotometer data x,y,z
uint32_t gps_read_inc        = 0;	// How often to read the GPS (600 = every 10 minutes, 0 = always)
uint32_t events_display_size = 20;	// Display events after recieving X events

// Siesmic event trigger parameters

uint32_t accelr_event_threshold = 2;	// Trigger level for siesmic events in milli-g
uint32_t accelr_event_cutoff_fr = 30;	// Siesmic event cutoff frequency

// Commands can be sent over the serial line to configure the display rates or whatever

typedef enum {
	NOOP,	// No-operation
	HELP,	// Help
	HTUX,	// Reset HTU chip
	HTUD,	// HUT display rate
	BMPD,	// BMP display rate
	LOCD,	// Location display rate
	TIMD,	// Timing display rate
	STSD,	// Status display rate
	EVQT,	// Event queue dump threshold

	ACLD,	// Accelerometer display rate
	MAGD,	// Magnetometer display rate

	ACLT,	// Accelerometer event threshold

	CMDS };	// Command count

typedef struct {
	int   Id;		// Command ID number
	void  (*proc)(int arg);	// Function to call
	char *Name;		// Command name
	char *Help;		// Command help text
	int   Par;		// Command parameter flag
} CmdStruct;

// Function forward references 

void noop(int arg);
void help(int arg);
void htux(int arg);
void htud(int arg);
void bmpd(int arg);
void locd(int arg);
void timd(int arg);
void stsd(int arg);
void evqt(int arg);
void acld(int arg);
void magd(int arg);
void aclt(int arg);

// Command table

CmdStruct cmd_table[CMDS] = {
	{ NOOP, noop, "NOOP", "Do nothing", 0 },
	{ HELP, help, "HELP", "Display commands", 0 },
	{ HTUX, htux, "HTUX", "Reset the HTU chip", 0 },
	{ HTUD, htud, "HTUD", "HTU Temperature-Humidity display rate", 1 },
	{ BMPD, bmpd, "BMPD", "BMP Temperature-Altitude display rate", 1 },
	{ LOCD, locd, "LOCD", "Location latitude-longitude display rate", 1 },
	{ TIMD, timd, "TIMD", "Timing uptime-frequency-utc display rate", 1 },
	{ STSD, stsd, "STSD", "Status info display rate", 1 },
	{ EVQT, evqt, "EVQT", "Event queue dump threshold", 1 },
	{ ACLD, acld, "ACLD", "Accelerometer display rate", 1 },
	{ MAGD, magd, "MAGD", "Magnatometer display rate", 1 },
	{ ACLT, aclt, "ACLT", "Accelerometer event trigger threshold", 1 }
};

#define CMDLEN 32
static char cmd[CMDLEN];		// Command input buffer
static int irdp=0, irdy=0, istp=0;	// Read, ready, stop

static char txtb[TBLEN];		// Text ring buffer
static uint32_t txtw = 0, txtr = 0, 	// Write and Read indexes
		tsze = 0, terr = 0;	// Buffer size and error code

typedef enum { TXT_NOERR=0, TXT_TOOBIG=1, TXT_OVERFL=2 } TxtErr;

#define TXTLEN 256
static char txt[TXTLEN];		// For writing to serial	
	 
// Initialize the timer chips to measure time between the PPS pulses and the EVENT pulse
// The PPS enters pin D2, the PPS is forwarded accross an isolating diode to pin D5
// The event pulse is also connected to pin D5. So D5 sees the LOR of the PPS and the
// event, while D2 sees only the PPS. In this way we measure the frequency of the
// clock MCLK/2 each second on the first counter, and the time between EVENTs on the second
// I use a the unconnected timer block TC1 to make a PLL that is kept in phase by the PPS
// arrival in TC0 and which is loaded with the last measured PPS frequency. This PLL will
// take over the PPS generation if the real PPS goes missing.

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
        TC_SetRC(TC1, 0, 42000000);			// One second approx initial PLL value
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

static uint32_t	rega0 = 0, 	// RA reg
		stsr0 = 0,	// Interrupt status register
		ppcnt = 0,	// PPS count
		delcn = 0;	// Synthetic PPS ms

static uint32_t	rega1, stsr1 = 0;

static uint32_t stsr2 = 0;

boolean pll_flag = false;	

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
	
        TC_SetRC(TC1, 0, rega0);		// Set the PLL count to what we just counted

	SwapBufs();				// Every PPS swap the read/write buffers
	ppcnt++;				// PPS count
	displ = 1;				// Display stuff in the loop
	gps_ok = true;				// Its OK because we got a PPS	
	pll_flag = true;			// Inhibit PLL, dont take over PPS arrived

#if PPS_PIN		
	digitalWrite(PPS_PIN,HIGH);
	digitalWrite(PPS_PIN,LOW);
#endif	
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

void SwapBufs() {
	struct Event *tbuf;			// Temp event buf pointer
	char *tdtm;				// Temp date/time string pointer
	tbuf = rbuf; rbuf = wbuf; wbuf = tbuf;	// Swap write with read buffer
	ridx = widx; widx = 0;			// Write count to read, reset the write count
	tdtm = rdtm; rdtm = wdtm; wdtm = tdtm;	// And swap asociated buffer date/time
}

// Handle isolated PPS (via diode) LOR with the Event
// The diode is needed to block Event pulses getting back to TC0
// LOR means Logical inclusive OR

void TC6_Handler() {

	if (widx < PPS_EVENTS) {	// Up to PPS_EVENTS stored per PPS
			
		// Read the latched tick count getting the event time
		// and then pull the ADC pipe line

		wbuf[widx].Tks = TC2->TC_CHANNEL[0].TC_RA;
		AdcPullData(&wbuf[widx]);
		widx++;
	}
	rega1 = TC2->TC_CHANNEL[0].TC_RA;	// Read the RA on channel 1 (PPS period)
	stsr1 = TC_GetStatus(TC2, 0); 		// Read status clear load bits

#if EVT_PIN
	digitalWrite(EVT_PIN,HIGH);	// Event detected
	digitalWrite(EVT_PIN,LOW);
#endif
}

// Accelerometer setup
// These settings come from reading the LSM303DLH doccumentation, which is more than the
// author of the Adda_fruit library did. It is a badly written load of rubbish, I have
// been forced to correct some bugs and export some private methods. It was a touch and go
// descision wether to just not use the library at all and do it all here. For now I will
// use it with the bug corrections.

// The following setup makes the accelarometer compare G-forces against a threshold value, and
// latch the output registers until they are read. To avoid excessive interrupt rates the high
// pass filter has been configured to keep the frequency low. 

// N.B. It took me a day to find out that I needed to use Active low and Open drain on the INT1
// signal, otherwise the Adda_fruit module wont pass it on. Beware !!!
 
void AclSetup() {
	uint8_t tmp, val;

	if (!acl_ok) return;

#define PMD 0x20	// Normal power mode (PM0=1,PM1=0:Normal)
#define DRT 0x00	// Data rate 50 Hz (0x08 = 100Hz)
#define AEN 0x07	// XYZ Enabled

	val = PMD | DRT | AEN;
	acl.write8(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_CTRL_REG1_A, val);

#define HPE1 0x04	// High pass filter Int 1 on
#define HPCF 0x03       // High pass cut off frequency

	val = HPE1 | HPCF;
	acl.write8(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_CTRL_REG2_A, val);

#define LIR1 0x06	// Latch Int1 bit Data ready
#define LIR2 0x00	// Latch Int2 bit Data ready (0x20 Latch On)

#define IHL_OD 0xC0	// Interrupt active low, open drain (Argh !!!)

	val = LIR1 | LIR2 | IHL_OD;
	acl.write8(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_CTRL_REG3_A, val);

#define BDU_FS 0x80	// Block data and scale +-2g

	val = BDU_FS;
	acl.write8(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_CTRL_REG4_A, val);

#define XYZ_HI 0x2A	// Hi values ZHIE YHIE XHIE
#define AOI_6D 0x00	// 0xC0 would enable 6 directions
	
	val = XYZ_HI | AOI_6D;
	acl.write8(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_INT1_CFG_A, val);

	val = accelr_event_threshold & 0x7F;
	acl.write8(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_INT1_THS_A, val);

	// The chip make very wide pulses (100ms), the values on the rising and
	// falling edges are different !

	attachInterrupt(digitalPinToInterrupt(ACL_PIN),Acl_ISR,RISING);
}

// Magnatometer setup, again the Adda_fruit library was inadequate. 

void MagSetup() {
	uint8_t val;

	if (!mag_ok) return;

	val = 0;
	mag.write8(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_CRA_REG_M, val);

#define GAIN 0x80	// +- 4.0 Gauss

	val = GAIN;
	mag.write8(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_CRB_REG_M, val);

#define MODE 0x0	// 01=Single conversion mode

	val = MODE;
	mag.write8(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_MR_REG_M, val);
}

// This Accelerometer ISR

static uint32_t accl_icount = 0, accl_flag = 0;

void Acl_ISR() {

#define IA 0x40

	accl_icount++;
	accl_flag = AclReadStatus();
	PushVib();
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

#define ZH 0x20	// Z High
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
	REG_ADC_MR = 0x10380080;	// Free run as fast as you can
	REG_ADC_CHER = 3;		// Channels 0 and 1
}

// Pull all data (16 values) from the ADC into a buffer

uint8_t AdcPullData(struct Event *b) {
	
	int i;

	for (i=0; i<ADC_BUF_LEN; i++) {			// For all in ADC pipeline
		while((ADC->ADC_ISR & 0x01)==0);	// Wait for channel 0 (2.5us)
		b->Ch0[i] = (uint16_t) ADC->ADC_CDR[0];	// Read ch 0
		while((ADC->ADC_ISR & 0x02)==0);	// Wait for channel 1 (2.5us)
		b->Ch1[i] = (uint16_t) ADC->ADC_CDR[1];	// Read ch 1
	}
}

// Increment date time by one second when not using the GPS

int hour=0, minute=0, second=0;
float latitude = 0.0, longitude = 0.0, altitude = 0.0;

void IncDateTime() {

	if (second++ >= 60) {
		second = 0;
		if (minute++ >= 60) {
			minute = 0;
			if (hour++ >= 24) {
				hour = 0;
			}
		}
	}
	sprintf(wdtm,"%02d%02d%02d",hour,minute,second);
}

// This is the nmea data string from the GPS chip

#define GPS_STRING_LEN 256
static char gps_string[GPS_STRING_LEN + 1];

// This function is dependent on the GPS chip implementation
// It should return a date time string as described above
// So you need to re-implements this for whichever chip you are using
// Here I am using the addafruit GPS chip

boolean GpsDateTime() {

	int i = 0;
	while (Serial1.available()) {
		if (i < GPS_STRING_LEN) {
			gps_string[i++] = (char) Serial1.read();
			gps_string[i] = 0;
		} else i++;
	}
	if (gps.parse(gps_string)) {

		// I choose RMCGGA by default, and get the altitude but no date.
		// Its easy to get the date once the records arrive at the Python end.
		// The GPS altitude is far more accurate than the barrometric altitude.
		// Warning: The syntax can not be changed, we need an integer hhmmss

		altitude  = gps.altitude;	
		latitude  = gps.latitudeDegrees;	// Easy place to get location
		longitude = gps.longitudeDegrees;	// Works well in Google maps
		hour      = gps.hour;
		minute    = gps.minute;
		second	  = gps.seconds;
		sprintf(wdtm,"%02d%02d%02d",hour,minute,second);

		time_ok	  = true;
		return true;
	}
	return false;
}

// Get the date time either from GPS or calculate it

uint32_t nxtdtr = 0;	// Next DateTime read second number

void GetDateTime() {

	if (gps_ok) {						// If the GPS is up
		if (!time_ok) { 				// If I havn't read it ever
			if (GpsDateTime()) {			// read GPS time and check its OK
				nxtdtr = ppcnt + gps_read_inc;	// Next time to read
				return;
			}
		} else {
			if (ppcnt >= nxtdtr) {			// Time to read GPS again ?
				if (GpsDateTime()) {
					nxtdtr = ppcnt + gps_read_inc;
					return;
				}
			}
		}
	}
	
	// So if we are here then we didn't read the GPS, so just increment time

	IncDateTime();
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
	uint8_t		Lock;				// The queue spin lock (not needed here)
	struct EventBuf	Events[EVENT_QSIZE];		// Queued events 
} EventQueue;

static EventQueue event_queue;

// Put an event in an EventBuf on the queue, if the queue is full then the oldest event
// is thrown away and the "missed" event count is incremented

uint8_t PutQueue(struct EventBuf *ebuf) {

	EventQueue *q = &event_queue;

	while(q->Lock) {}; q->Lock = 1;		// Spin lock on queue
	q->Events[q->WrPtr] = *ebuf;		// Write event to the queue
	q->WrPtr = (q->WrPtr + 1) % EVENT_QSIZE;// Increment the write pointer
	if (q->Size < EVENT_QSIZE) q->Size++;	// If we are overwriting old enties that havnt been read
	else {
		q->Missed++;					// Say we missed some events
		q->RdPtr = (q->RdPtr + 1) % EVENT_QSIZE;	// and throw the oldest event away	
	}
	q->Lock = 0;
	return q->Missed;
}

// Pop an event off the queue, if the queue is empty nothing happens
// the queue size is zero when the queue is empty, and this is the
// return value

uint8_t PopQueue(struct EventBuf *ebuf) {	// Points to where the caller wants the event stored

	EventQueue *q = &event_queue;

	while(q->Lock) {}; q->Lock = 1;		// Spin lock on queue
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

// Arduino setup function, initialize hardware and software
// This is the first function to be called when the sketch is started

void setup() {

#if FLG_PIN
	pinMode(FLG_PIN, OUTPUT);	// Pin for the ppsfl flag for debug
#endif
#if EVT_PIN
	pinMode(EVT_PIN, OUTPUT);	// Pin for the cosmic ray event 
#endif
#if PPS_PIN
	pinMode(PPS_PIN, OUTPUT);	// Pin for the PPS (LED pin)
#endif

	pinMode(POW_ONE, OUTPUT);	// Breakout power pin
	pinMode(POW_TWO, OUTPUT);	// ditto

	Serial.begin(SERIAL_BAUD_RATE);	// Start the serial line
	Serial1.begin(GPS_BAUD_RATE);	// and the second

	gps.begin(GPS_BAUD_RATE);	// Chip baud rate

	gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);	// With altitude but no yy/mm/dd
	gps.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);	// each second

	InitQueue();			// Reset queue pointers, missed count, and size

	strcpy(rdtm,"");		// Set initial value for date/time
	strcpy(wdtm,"");

	// Power OFF/ON the breakouts

	digitalWrite(POW_ONE,LOW);	// Power off
	digitalWrite(POW_TWO,LOW);	
	delay(100);			// Wait for caps to discharge
	digitalWrite(POW_ONE,LOW);	// Power on
	digitalWrite(POW_TWO,LOW);	

	// Initialize breakouts

	htu_ok = htu.begin();
	bmp_ok = bmp.begin();
	acl_ok = acl.begin();
	mag_ok = mag.begin();
	dof_ok = dof.begin();

	AclSetup();
	MagSetup();
	AdcSetup();
	
	TimersStart();			// Start timers
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

void PushTxt(char *txt) {
	
	int i, l = strlen(txt);

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
		temph = htu.readTemperature();
		humid = htu.readHumidity();
		sprintf(txt,"{'HTU':{'Tmh':%5.3f,'Hum':%4.1f}}\n",temph,humid);
		PushTxt(txt);
	}
}

// Push BMP temperature and altitude from BMP chip

void PushBmp(int flg) {	// If flg is true always push

	double altib = 0.0;
	float  tempb = 0.0;
	float  presr = 0.0;
	sensors_event_t bmp_event;	// Barrometric pressure event		

	if ((flg) || ((bmp_ok) && ((ppcnt % alttmp_display_rate) == 0))) {
		bmp.getEvent(&bmp_event);
		if (bmp_event.pressure) {
			presr = bmp_event.pressure;
			bmp.getTemperature(&tempb);
			altib = bmp.pressureToAltitude((float) SENSORS_PRESSURE_SEALEVELHPA, 
							presr,tempb);
			sprintf(txt,"{'BMP':{'Tmb':%5.3f,'Prs':%5.3f,'Alb':%4.1f}}\n",tempb,presr,altib);
			PushTxt(txt);
		}
	}
}

// When the detector is shaken this outputs the (vcn) vibration count the (vax) axis bits
// the latched acceleration values for all 3 axis, the time it happened and the field
// strengths in all three axis. From all this information the Python monitor will be able
// to work out whats going on, sustained vibration or whatever. 

void PushVib() { // Push an event when shake detected => Earth Quake 

uint32_t old_icount = 0;

	if (accl_flag) {
		if (accl_icount != old_icount) {
			old_icount = accl_icount;
			PushTim(1);		// Push these first, and then vib
			PushAcl(1);		// This is the real latched value
			PushMag(1);
			sprintf(txt,"{'VIB':{'Vax':%d,'Vcn':%d}}\n",accl_flag,accl_icount);
			PushTxt(txt);
		}
	}
}

// Push the magnetic field strengths in all three axis in micro tesla (gauss)

void PushMag(int flg) {	// Push the mago stuff
	sensors_event_t mag_event;
	sensors_vec_t xyz;
	
	if ((flg) || ((mag_ok) && ((ppcnt % magnot_display_rate) == 0))) {
		mag.getEvent(&mag_event);

		// Micro Tesla

		sprintf(txt,"{'MAG':{'Mgx':%f,'Mgy':%f,'Mgz':%f}}\n",
			mag_event.magnetic.x,
			mag_event.magnetic.y,
			mag_event.magnetic.z);
		PushTxt(txt);

		// Orientation (Easy to calculate later in Python - dont waste resources)
#ifdef ORIENTATION
		if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &xyz)) {
			sprintf(txt,"{'MOG':{'Mox':%f,'Moy':%f,'Moz':%f}}\n",xyz.x,xyz.y,xyz.z);
			PushTxt(txt);
		}
#endif
	}
}

// Push the acceleration values in xyz in meters per sec squared

void PushAcl(int flg) { // Push the accelerometer and compass stuff
	sensors_event_t acl_event;
	sensors_vec_t xyz; 

	if ((flg) || ((acl_ok) && ((ppcnt % accelr_display_rate) == 0))) {
		acl.getEvent(&acl_event);

		// Meters per second squared

		sprintf(txt,"{'ACL':{'Acx':%f,'Acy':%f,'Acz':%f}}\n",
			acl_event.acceleration.x,
			acl_event.acceleration.y,
			acl_event.acceleration.z);
		PushTxt(txt);

		// Orientation (Easy to calculate later in Python - dont waste resources)
#ifdef ORIENTATION		
		if (dof.accelGetOrientation(&acl_event, &xyz)) {
			sprintf(txt,"{'AOL':{'Aox':%f,'Aoy':%f,'Aoz':%f}}\n",xyz.x,xyz.y,xyz.z);
			PushTxt(txt);
		}
#endif
	}
}

// Push location latitude longitude in degrees so that google maps gets it correct

void PushLoc(int flg) {
		
	if ((flg) || ((ppcnt % latlon_display_rate) == 0)) {
		sprintf(txt,"{'LOC':{'Lat':%f,'Lon':%f,'Alt':%f}}\n",latitude,longitude,altitude);
		PushTxt(txt);
	}
}

// Push timing

void PushTim(int flg) {

	if ((flg) || ((ppcnt % frqutc_display_rate) == 0)) {
		sprintf(txt,"{'TIM':{'Upt':%4d,'Frq':%7d,'Sec':%s}}\n",ppcnt,rega0,rdtm);
		PushTxt(txt);
	}			
}

// Push status

void PushSts(int flg, int qsize, int missed) {
uint8_t res;

	if ((flg) || ((ppcnt % status_display_rate) == 0)) {
		sprintf(txt,"{'STS':{'Qsz':%2d,'Mis':%2d,'Ter':%d,'Htu':%d,'Bmp':%d,'Acl':%d,'Mag':%d,'Gps':%d}}\n",
			qsize,missed,terr,htu_ok,bmp_ok,acl_ok,mag_ok,gps_ok);
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
		for (j=0; j<ADC_BUF_LEN; j++) {		// Copy accross ADC values
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

			sprintf(txt,
				"{'EVT':{'Evt':%1d,'Frq':%8d,'Tks':%8d,'Etm':%s%s,"
				"'Adc':[[%d,%d,%d,%d,%d,%d,%d,%d],[%d,%d,%d,%d,%d,%d,%d,%d]]}}\n",
				eb.Count, eb.Frequency, eb.Ticks, eb.DateTime, index(stx,'.'),
				eb.Ch0[0],eb.Ch0[1],eb.Ch0[2],eb.Ch0[3],eb.Ch0[4],eb.Ch0[5],eb.Ch0[6],eb.Ch0[7],
				eb.Ch1[0],eb.Ch1[1],eb.Ch1[2],eb.Ch1[3],eb.Ch1[4],eb.Ch1[5],eb.Ch1[6],eb.Ch1[7]);
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

void help(int arg) {	// Display the help
	int i;
	CmdStruct *cms;

	for (i=0; i<CMDS; i++) {
		cms = &cmd_table[i];
		sprintf(txt,"%s(%d) - %s\n",cms->Name,cms->Par,cms->Help);
		PushTxt(txt);
	}
}

void htux(int arg) { htu_ok = htu.begin(); }
void htud(int arg) { humtmp_display_rate = arg; }
void bmpd(int arg) { alttmp_display_rate = arg; }
void locd(int arg) { latlon_display_rate = arg; }
void timd(int arg) { frqutc_display_rate = arg; }
void stsd(int arg) { status_display_rate = arg; }

void evqt(int arg) { 
	events_display_size = arg % EVENT_QSIZE; 
	if (events_display_size == 0)
		events_display_size = 24;
}

void acld(int arg) { accelr_display_rate = arg; }
void magd(int arg) { magnot_display_rate = arg; }

void aclt(int arg) { 
	uint8_t val = 0;
	accelr_event_threshold = arg & 0x7F; 
	val = accelr_event_threshold;
	acl.write8(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_INT1_THS_A, val);
}

// Look up a command in the command table for the given command string
// and call it with its single integer parameter

void ParseCmd() {
	int i, p=0, cl=0;
	char *cp, *ep;
	CmdStruct *cms;

	for (i=0; i<CMDS; i++) {
		cms = &cmd_table[i];
		cl = strlen(cms->Name);
		if (strncmp(cms->Name,cmd,cl) == 0) {
			if ((cms->Par) && (strlen(cmd) > cl)) {
				cp = &cmd[cl];
				p = (int) strtoul(cp,&ep,0);
			}
			cms->proc(p);
			break;
		}
	}
}

// This waits for a ready buffer from ReadOneChar. Once ready the buffer is
// locked until its been seen here

void DoCmd() {
	if (irdy) {
		if (irdp) {
			sprintf(txt,"{'CMD':%s}\n",cmd);
			PushTxt(txt);
			ParseCmd();
		}
		bzero((void *) cmd, CMDLEN);
		irdp = 0; irdy = 0; istp = 0;
	}
}

// Arduino main loop does all the user space work

void loop() {

	int missed, qsize;	// Queue vars

	// If the GPS is not locked yet (no PPS at startup) print status
#if 0
	if (time_ok == false) {
		delay(1);
		if (delcn++ >= 1000) {		// Wait for a second approw
			delcn = 0;		// Reset delay
			GetDateTime();		// Read the next date/time from the GPS chip
			PushSts(0,qsize,missed);// Push status
		}
	}

	else 
#endif
	if (displ) {			// Displ is set in the PPS ISR, we will reset it here
		DoCmd();			// Execute any incomming commands
		PushEvq(0,&qsize,&missed);	// Push any events
		PushHtu(0);			// Push HTU temperature and humidity
		PushBmp(0);			// Push BMP temperature and barrometric altitude
		PushLoc(0);			// Push location latitude and longitude
		PushTim(0);			// Push timing data
		PushMag(0);			// Push mago data
		PushAcl(0);			// Push accelarometer data
		PushSts(0,qsize,missed);	// Push status
		GetDateTime();		// Read the next date/time from the GPS chip
		displ = 0;			// Clear flag for next PPS			
	}
	
	PutChar();	// Print one character per loop !!!
	ReadOneChar();	// Get next input command char
}
