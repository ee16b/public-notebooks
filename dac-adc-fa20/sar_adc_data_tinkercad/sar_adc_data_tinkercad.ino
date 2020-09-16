/* Set the serial connection speed */
#define BAUD_RATE       9600

/* Set the simulated clock period in miliseconds */
#define CLOCK_PERIOD    10000000

/* Set which bits correspond to which pins */
#define BIT_3           7
#define BIT_2           8
#define BIT_1           12
#define BIT_0           13
#define READ_PIN        A0

/* Set the feedback pin from which to read the output of the comparator */
#define V_IN            A1

int result = 0;

void setup() {
  // Start a serial connection to send data to the computer
  Serial.begin(BAUD_RATE);

  // Setup the digital pins as outputs
  pinMode(BIT_3, OUTPUT);
  pinMode(BIT_2, OUTPUT);
  pinMode(BIT_1, OUTPUT);
  pinMode(BIT_0, OUTPUT);
  pinMode(READ_PIN, INPUT);

  // Set the initial values of these pins to LOW (0V)
  digitalWrite(BIT_3, LOW);
  digitalWrite(BIT_2, LOW);
  digitalWrite(BIT_1, LOW);
  digitalWrite(BIT_0, LOW);

  float conversion_factor = calibrate();
  Serial.print("Conversion factor: ");
  Serial.println(conversion_factor,DEC);
  Serial.print("Setup done\n");
}

void loop () {




  // cycle 1: clean all
  digitalWrite(BIT_3, LOW);
  digitalWrite(BIT_2, LOW);
  digitalWrite(BIT_1, LOW);
  digitalWrite(BIT_0, LOW);
  result = 0;

  delayMicroseconds(CLOCK_PERIOD);




  // cycle 2: test BIT_3
  digitalWrite(BIT_3, HIGH);
  delayMicroseconds(CLOCK_PERIOD / 2);
  if (digitalRead(V_IN)) {
    result += 8;
  } else {
    digitalWrite(BIT_3, LOW);
  }
  delayMicroseconds(CLOCK_PERIOD / 2);



  // cycle 3: test BIT_2
  digitalWrite(BIT_2, HIGH);
  delayMicroseconds(CLOCK_PERIOD / 2);
  if (digitalRead(V_IN)) {
    result += 4;
  } else {
    digitalWrite(BIT_2, LOW);
  }
  delayMicroseconds(CLOCK_PERIOD / 2);


  // cycle 4: test BIT_1
  digitalWrite(BIT_1, HIGH);
  delayMicroseconds(CLOCK_PERIOD / 2);
  if (digitalRead(V_IN)) {
    result += 2;
  } else {
    digitalWrite(BIT_1, LOW);
  }
  delayMicroseconds(CLOCK_PERIOD / 2);




  // cycle 5: test BIT_0
  digitalWrite(BIT_0, HIGH);
  delayMicroseconds(CLOCK_PERIOD / 2);
  if (digitalRead(V_IN)) {
    result += 1;
  } else {
    digitalWrite(BIT_0, LOW);
  }
  delayMicroseconds(CLOCK_PERIOD / 2);

  // print result to serial monitor
  // Serial.println(result, BIN);
  int_to_bin(result);


}

void int_to_bin(int var) {
  for (unsigned int test = 0x8; test; test >>= 1) {
    Serial.write(var  & test ? '1' : '0');
  }
  Serial.println();
}

float calibrate() {
  digitalWrite(BIT_0, HIGH);
  digitalWrite(BIT_1, HIGH);
  digitalWrite(BIT_2, HIGH);
  digitalWrite(BIT_3, HIGH);
  delay(10);
  float max_sensor = analogRead(READ_PIN);
  float conversion_factor = 3.3/max_sensor;
  digitalWrite(BIT_0, LOW);
  digitalWrite(BIT_1, LOW);
  digitalWrite(BIT_2, LOW);
  digitalWrite(BIT_3, LOW);
  return conversion_factor;
  }
