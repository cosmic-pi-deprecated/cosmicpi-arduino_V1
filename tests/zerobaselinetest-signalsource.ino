/*
This is a modified version of blink, run on an Arduino Leonardo at 3.3V
It is connected via a diode to pin 5 on the test units for the zero baseline test
It generates a 10ms pulse at a rate of 1Hz, up to 10Hz.
Randomness was chosen as a way of checking the full systematic range,
rather than relying on a periodic pulse which would have drift and not test full range sensitivity.
Responses <90ms after the falling edge weren't tested, but we can look at this later.
 */
int x = 0; //integer for the delay between pulses

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin 13 as an output.
  pinMode(13, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(10);              // wait for 10ms
  digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
x=random(90,990);//set x to be a random delay from 100 to 1000
  delay(x);              // wait for a second
}
