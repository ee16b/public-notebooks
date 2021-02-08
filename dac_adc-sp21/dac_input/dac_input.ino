/* Set the serial connection speed */
#define BAUD_RATE       9600

/* Set which bits correspond to which pins */
#define BIT_3           P3_2
#define BIT_2           P2_7
#define BIT_1           P4_2
#define BIT_0           P4_1
#define READ_PIN        P6_0

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
  max_voltage = 3.3*((1<<BITS) - 1)/(1<<BITS);
  calibrate();
  reset_blinker();
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
  Serial.print("Received input bit string: "); Serial.print(serial_buffer);

  if (serial_buffer.length() == 4) {
    for (int i = 0; i < 3; i++ ) {
//      Serial.print("Bit Input at index "); Serial.print(i); Serial.print(": "); Serial.println(serial_buffer[i]);
      if (serial_buffer[i] == '1') {
        digitalWrite(pins[i+1], HIGH);
      } else {
        digitalWrite(pins[i+1], LOW);
      }
    }
  } else if (serial_buffer.length() == 5) {
    for (int i = 0; i < 4; i++ ) {
//      Serial.print("Bit Input at index "); Serial.print(i); Serial.print(": "); Serial.println(serial_buffer[i]);
      if (serial_buffer[i] == '1') {
        digitalWrite(pins[i], HIGH);
      } else {
        digitalWrite(pins[i], LOW);
      }
    }
  }
  serial_buffer = "";
  plot();
  delay(50);
  
  
  
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
