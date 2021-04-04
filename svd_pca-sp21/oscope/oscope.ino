/*
 * oscope.ino
 *
 * Displays voltage output, peak-to-peak voltage, and
 * a constant threshold voltage on the serial plotter.
 *
 * EECS16B Spring 2021
 * Rafael Calleja
 *
 */


#define BAUD_RATE         9600

#define MIC_IN            P6_0
#define PEAK_DTCTR        P6_1
#define BUTTON            PUSH1 //P2.1, bottom left of board

#define CALIBRATION_TIME  1000
#define THRESHOLD         2.5
#define DEFAULT_P2P       0
#define MAX_VOLTAGE       3.45 //3.3V voltage regulator reading from your multimeter (will probably be between 3.3-3.45V
#define CONVERSION_FACTOR MAX_VOLTAGE/4096
#define MAX_PEAK          2.3


float half_max = MAX_VOLTAGE/2;


void setup() {

  Serial.begin(BAUD_RATE);

  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(MIC_IN, INPUT);
  pinMode(BUTTON, INPUT_PULLUP);
  reset_blinker();
  Serial.println();
  Serial.println();
  
}

void loop() {
  // turn LED off:
  digitalWrite(RED_LED, LOW);

  
  float val = read_mic(MIC_IN);
  float peak_to_peak = read_mic(PEAK_DTCTR);
  plot(val, peak_to_peak);

}

float read_mic(int pin) {
  float v_out_int = analogRead(pin);
//  Serial.println(v_out_int);
  float v_out = v_out_int*CONVERSION_FACTOR;
  return v_out;

}

void plot(float val, float peak_to_peak) {
  float p2p = min(MAX_VOLTAGE, max(2*peak_to_peak - MAX_VOLTAGE+0.6, 0)*MAX_VOLTAGE/MAX_PEAK);
  Serial.print("MIC_OUT:"); Serial.print(val - half_max,2); Serial.print(",");
  Serial.print("2.5V_THRESHOLD:"); Serial.print(THRESHOLD,2); Serial.print(",");
  Serial.print("P2P-"); Serial.print(p2p,2), Serial.print(":"); Serial.println(p2p,2);
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
