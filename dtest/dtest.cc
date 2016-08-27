
// Design test firmware
// Julian Lewis lewis.julian@gmail.com

#define SERIAL_BAUD_RATE 9600   // Serial line 
#define PIN_TRIG_1 13		// Event trigger pin Ch 1
#define PIN_TRIG_2 12		// Ch 2
#define PIN_PPS_IN 10		// PPS input

#define DAC_MIN 0.00560 	// Minimum voltage
#define DAC_MAX 2.78		// Maximum voltage
#define DAC_08W 0.0188		// 8 Bit weight (2.78-0.0056)/255
#define DAC_12W 0.00067734375	// 12 bit weight (2.78-0.0056)/4096
#define DAC_08M 0xFF		// 8 bit mask
#define DAC_12M 0xFFF		// 12 bit mask
#define DAC_08R 8		// 8 bit resolution
#define DAC_12R 12		// 12 bit resolution

// Choose DAC resolution 8 or 12

#define DAC_RES DAC_08R		// DAC resolution in bits
#define DAC_MSK DAC_08M		// DAC bit mask
#define DAC_WGH DAC_08W		// DAC bit weight in volts

// The time increment between DAC outputs
// The ADC runs at 2.5 microseconds per conversion

#define INC_TUS	5		// Time increment in microseconds

#define TXT_LEN 256
static char txt[TXT_LEN];

// Make a pulse vector

void p_vector(	uint16_t tp,	// trigger pulse flag
		uint16_t cv,	// Current dac value
		uint16_t us, 	// Time in microseconds of vector
		uint16_t fv) {	// Final dac value

	float nv = us/INC_TUS;		// Number of values to send
	float vi = (fv - cv) / nv;	// Voltage increment
	float dv = cv;			// DAC voltage value
	int i, j = nv;

	if (tp) {
		digitalWrite(PIN_TRIG_1,HIGH);
		digitalWrite(PIN_TRIG_2,HIGH);
		delay(1);
		digitalWrite(PIN_TRIG_1,LOW);
		digitalWrite(PIN_TRIG_2,LOW);
		tp = 0;
	}

	for (i=0; i<=j; i++) {
		analogWrite(DAC0,dv);
		dv = dv + vi;
	}
}

// Make a pulse

void p_out(	uint16_t rus,	// Rise time in us 
		uint16_t rfv, 	// Rise final value
		uint16_t tus, 	// Flat top in us
		uint16_t fus) {	// Fall in us

	p_vector(1,0,rus,rfv);
	p_vector(0,rfv,tus,rfv);
	p_vector(0,rfv,fus,0);
}


void setup() {
	Serial.begin(SERIAL_BAUD_RATE); // Start the serial line
	pinMode(PIN_TRIG_1,OUTPUT);
	pinMode(PIN_TRIG_2,OUTPUT);
	pinMode(PIN_PPS_IN,INPUT);
}

static int count = 0;

void loop() {

	while (digitalRead(PIN_PPS_IN) == 0) {};
	
	//delay(10);
	digitalWrite(PIN_TRIG_1,HIGH);
	digitalWrite(PIN_TRIG_1,LOW);

	delay(200);
}
