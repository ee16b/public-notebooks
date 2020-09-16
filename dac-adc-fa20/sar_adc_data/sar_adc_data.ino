/* Set the serial connection speed */
#define BAUD_RATE       9600

/* Set the simulated clock period in miliseconds */
#define CLOCK_PERIOD    100
#define CONV            0.0008616188
/* Set which bits correspond to which pins */
#define BIT_3           P3_2
#define BIT_2           P2_7
#define BIT_1           P4_2
#define BIT_0           P4_1
#define READ_PIN        P6_0

/* Set the feedback pin from which to read the output of the comparator */
#define V_IN            P6_5

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
  reset_blinker();
}

void loop () {

  // cycle 1: clean all
  digitalWrite(BIT_3, LOW);
  digitalWrite(BIT_2, LOW);
  digitalWrite(BIT_1, LOW);
  digitalWrite(BIT_0, LOW);
  result = 0;

  Serial.println("Values: ");
  Serial.print("[");
  
  delayMicroseconds(CLOCK_PERIOD);
  plot();


  // cycle 2: test BIT_3
  digitalWrite(BIT_3, HIGH);
  delayMicroseconds(CLOCK_PERIOD / 2);
  plot();
  if (digitalRead(V_IN)) {
    result += 8;
  } else {
    digitalWrite(BIT_3, LOW);
  }
  delayMicroseconds(CLOCK_PERIOD / 2);
  plot();


  // cycle 3: test BIT_2
  digitalWrite(BIT_2, HIGH); 
  delayMicroseconds(CLOCK_PERIOD / 2);
  plot();
  if (digitalRead(V_IN)) {
    result += 4;
  } else {
    digitalWrite(BIT_2, LOW);
  }
  delayMicroseconds(CLOCK_PERIOD / 2);
  plot();

 
  // cycle 4: test BIT_1
  digitalWrite(BIT_1, HIGH);
  delayMicroseconds(CLOCK_PERIOD / 2);
  plot();
  if (digitalRead(V_IN)) {
    result += 2;
  } else {
    digitalWrite(BIT_1, LOW);
  }
  delayMicroseconds(CLOCK_PERIOD / 2);
  plot();


  // cycle 5: test BIT_0
  digitalWrite(BIT_0, HIGH);
  delayMicroseconds(CLOCK_PERIOD / 2);
  plot();
  if (digitalRead(V_IN)) {
    result += 1;
  } else {
    digitalWrite(BIT_0, LOW);
  }
  delayMicroseconds(CLOCK_PERIOD / 2);
  plot();
  
  Serial.println("0.000000000]");
  
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

void plot() {
  float v_out_int = analogRead(READ_PIN); //Uncomment this line to collect data
  float v_out = v_out_int*CONV; //unco
  Serial.print(v_out,DEC);
  Serial.print(",");
}
