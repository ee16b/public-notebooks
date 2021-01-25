/* Set the serial connection speed */
#define BAUD_RATE       9600

/* Set which bits correspond to which pins */
#define SQ_WV           P3_2

void setup() {
  pinMode(SQ_WV, OUTPUT);
  reset_blinker();
}

// Create a 0.25Hz square wave with 50% duty cycle and 3.3Vpp
void loop() {
  digitalWrite(SQ_WV, HIGH);   // set the square wave to 3.3V
  delay(2000);                 // wait for a second
  digitalWrite(SQ_WV, LOW);    // set the square wave to 0V
  delay(2000);                 // wait for a second
}

// Reset success indicator
void reset_blinker() {
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  digitalWrite(RED_LED, HIGH);
  delay(100);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  delay(100);
  digitalWrite(RED_LED, HIGH);
  digitalWrite(GREEN_LED, LOW);
  delay(100);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  delay(100);
  digitalWrite(GREEN_LED, LOW);
}
