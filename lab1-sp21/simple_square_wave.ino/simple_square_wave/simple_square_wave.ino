/* Set the serial connection speed */
#define BAUD_RATE       9600

/* Set which bits correspond to which pins */
#define SQ_WV           P3_2

void setup() {
  pinMode(SQ_WV, OUTPUT);
}

// Create a 1Hz square wave with 50% duty cycle and 3.3Vpp
void loop() {
  digitalWrite(SQ_WV, HIGH);  // set the square wave to 3.3V
  delay(2000);               // wait for a second
  digitalWrite(SQ_WV, LOW);    // set the square wave to 0V
  delay(2000);               // wait for a second
}
