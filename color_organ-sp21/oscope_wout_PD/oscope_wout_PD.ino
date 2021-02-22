/*
 * oscope_wout_PD.ino
 *
 * Displays voltage output, peak-to-peak voltage, and
 * a constant threshold voltage on the serial plotter.
 *
 * EE16B Summer 2020
 * Mia Mirkovic
 *
 */


#define BAUD_RATE         9600

#define MIC_IN            P6_0
#define CALIB_PIN         P6_2
#define BUTTON            PUSH1 //P2.1, bottom left of board

#define CALIBRATION_TIME  1000
#define DELAY             200 //peak-to-peak calculation time in ms
#define THRESHOLD         2.5
#define DEFAULT_P2P       0
#define MAX_VOLTAGE       //3.3V voltage regulator reading from your multimeter (will probably be between 3.3-3.4V)

int buttonState = 0;
float conversion_factor = 0;
float peak_to_peak = 0;
float half_max = MAX_VOLTAGE/2;


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
    Serial.print("Voltage Threshold Peak-to-Peak");
    Serial.println(peak_to_peak,DEC);


    float val = read_mic(conversion_factor);

    int t_init = millis();
    int t = millis();
    float max_val = 1;
    float min_val = 3.3;
    float pre_peak = peak_to_peak;

    while (t < t_init+DELAY){
      val = read_mic(conversion_factor);
      max_val = check_max(val, max_val);
      min_val = check_min(val, min_val);
      peak_to_peak = check_p2p();
      plot(val, peak_to_peak);
      t = millis();
    }
    peak_to_peak = max_val-min_val;
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
  float max_sensor = analogRead(CALIB_PIN);
  float conversion_factor = 3.3/max_sensor;
  return conversion_factor;
  }

float read_mic(float conv) {
  float v_out_int = analogRead(MIC_IN);
  float v_out = v_out_int*conv;
  return v_out;

}

void plot(float val, float peak_to_peak) {
  Serial.print("MIC_OUT:"); Serial.print(val - half_max,2); Serial.print(",");
  Serial.print("2.5V_THRESHOLD:"); Serial.print(THRESHOLD,2); Serial.print(",");
  Serial.print("P2P-"); Serial.print(peak_to_peak,2), Serial.print(":"); Serial.println(peak_to_peak,2);
}

void check_p2p(float peak_to_peak) {}
  if (peak_to_peak < 0) {
    return -1*peak_to_peak;
  }
  return peak_to_peak
}

void check_max(float val, float max_val){
  if (val > max_val){
     return val;
  } else {
    return max_val;
  }
}

void check_min(float val, float min_val) {
  if (val < min_val && val >= 0){
    return val;
  }
  return min_val;
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
