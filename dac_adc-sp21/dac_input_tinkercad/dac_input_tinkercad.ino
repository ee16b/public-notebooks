/* Set the serial connection speed */
#define BAUD_RATE       9600

/* Set which bits correspond to which pins */
#define BIT_3           7
#define BIT_2           8
#define BIT_1           12
#define BIT_0           13
#define READ_PIN        A0

/* Set number of bits */
#define BITS            3

/* Set how long to delay in between digital outputs */
#define DELAY           500

float max_voltage;
float conversion_factor;
String serial_buffer;
uint8_t pins[4] = {BIT_3, BIT_2, BIT_1, BIT_0};

void setup()
{
  // Start a serial connection to send data to the computer
  Serial.begin(BAUD_RATE);

  // Set pins to output mode
  pinMode(BIT_3, OUTPUT);
  pinMode(BIT_2, OUTPUT);
  pinMode(BIT_1, OUTPUT);
  pinMode(BIT_0, OUTPUT);
  pinMode(READ_PIN, INPUT);
  max_voltage = 5.0*((1<<BITS) - 1)/(1<<BITS);
  Serial.print("Max voltage: "); Serial.println(max_voltage);
  calibrate();
  Serial.println("Input bit values in decreasing bit order\n(e.g. 3-bit: MSB -> '010' -> LSB ; 4-bit: MSB -> '0101' -> LSB)\n");
}

void loop()
{
  while(!Serial.available()) {

  }
  while(Serial.available()) {
    delay(2);
    char c = Serial.read();
    serial_buffer += c;
  }
  Serial.print("Received input bit string: "); Serial.println(serial_buffer);
  //Serial.print("Input bit string length: "); Serial.println(serial_buffer.length());

  if (serial_buffer.length() == 3) {
    for (int i = 0; i < 3; i++ ) {
      //Serial.print("Writing pin "); Serial.print(pins[i]); Serial.print(" to ");
      if (serial_buffer[i] == '1') {
        digitalWrite(pins[i+1], HIGH);
        Serial.println("HIGH.");
      } else {
        digitalWrite(pins[i+1], LOW);
        Serial.println("LOW.");
      }
    }
  } else if (serial_buffer.length() == 4) {
    for (int i = 0; i < 4; i++ ) {
      Serial.print("Writing pin "); Serial.print(pins[i]); Serial.print(" to ");
      if (serial_buffer[i] == '1') {
        digitalWrite(pins[i], HIGH);
        Serial.println(" HIGH.");
      } else {
        digitalWrite(pins[i], LOW);
        Serial.println(" LOW.");
      }
    }
  }
  serial_buffer = "";
  plot();
  delay(50);



}


float calibrate() {
  digitalWrite(BIT_0, HIGH);
  digitalWrite(BIT_1, HIGH);
  digitalWrite(BIT_2, HIGH);
  digitalWrite(BIT_3, HIGH);
  delay(100);
  float max_sensor = analogRead(READ_PIN);
  digitalWrite(BIT_0, LOW);
  digitalWrite(BIT_1, LOW);
  digitalWrite(BIT_2, LOW);
  digitalWrite(BIT_3, LOW);
  conversion_factor = max_voltage/max_sensor;
  }

  void plot() {
    float v_out_int = analogRead(READ_PIN);
    float v_out = v_out_int*conversion_factor;
    Serial.print("DAC:"); Serial.print(v_out,DEC); Serial.print(",");
    Serial.println();
  }
