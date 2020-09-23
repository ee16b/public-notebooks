/*
 * tune_mic.ino
 *
 * Displays voltage output, peak-to-peak voltage, and
 * a constant threshold voltage on the serial plotter.
 *
 * EE16B Summer 2020
 * Mia Mirkovic
 *
 */


#define BAUD_RATE         9600

#define CAL_PROBE         P6_2
#define PROBE1            P6_0
#define PROBE2            P6_1
#define BUTTON            PUSH1 //P2.1, bottom left of board

#define CALIBRATION_TIME  1000
#define DELAY             200 //peak-to-peak calculation time in ms
#define THRESHOLD         2.5
#define DEFAULT_P2P       0

int buttonState = 0;
float conversion_factor = 0;
float peak_to_peak = 0;


void setup() {

  Serial.begin(BAUD_RATE);

  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(MIC_IN, INPUT);
  pinMode(BUTTON, INPUT_PULLUP);
  reset_blinker();
  conversion_factor = calibrate();

}

void loop() {
  // put your main code here, to run repeatedly:
  buttonState = digitalRead(BUTTON);


  // check if the pushbutton is pressed.
  // if it is, the buttonState is HIGH:
  if (buttonState == LOW) {
    conversion_factor = calibrate();

  }
  else {
    // turn LED off:
    digitalWrite(RED_LED, LOW);
    Serial.print("Probe1 Probe2");
    Serial.println(peak_to_peak,DEC);

    float probe1 = read_probe(PROBE1, conversion_factor);
    float probe2 = read_probe(PROBE2, conversion_factor);

    int t_init = millis();
    int t = millis();

    while (t < t_init+DELAY){
      probe1 = read_mic(PROBE1, conversion_factor);
      probe2 = read_probe(PROBE2, conversion factor);
      plot();
      t = millis();
    }
  }
}

float calibrate(){
  int running_out = 3*CALIBRATION_TIME/4;

  // turn LED on:
  digitalWrite(RED_LED, HIGH);
  int t_init = millis();
  int t = millis();
  Serial.println("Calibrating...");
  while (t < t_init+running_out){
    conversion_factor = get_conv();
    digitalWrite(GREEN_LED, HIGH);
    delay(200);
    digitalWrite(GREEN_LED, LOW);
    delay(200);
    t = millis();
   }

   Serial.println("Almost done...");

   while (t < t_init+CALIBRATION_TIME){
    conversion_factor = get_conv();
    digitalWrite(GREEN_LED, HIGH);
    delay(100);
    digitalWrite(GREEN_LED, LOW);
    delay(100);
    t = millis();
  }

  Serial.print("Calibrated. Conversion factor: ");
  return conversion_factor;

}

float get_conv() {
  float max_sensor = analogRead(CALPROBE);
  float conversion_factor = 3.3/max_sensor;
  return conversion_factor;
  }

float read_probe(int probe, float conv) {
  float v_out_int = analogRead(probe);
  float v_out = v_out_int*conv;
  return v_out;

}

void plot() {
  Serial.print(probe1,DEC);
  Serial.print(" ");
  Serial.println(probe1, DEC);
}


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
