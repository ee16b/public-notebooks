/*
 * closed_loop.ino
 *
 * EE16B Fall 2016
 * John Maidens, Emily Naviasky & Nathaniel Mailoa
 *
 * EE16B Fall 2017
 * Andrew Blatner
 * 
 * * EE16B Spring 2020
 * Kourosh Hakhamaneshi
 */

 #include <MspFlash.h>

#define LEFT_MOTOR                  P2_0
#define LEFT_ENCODER                P6_1
#define RIGHT_MOTOR                 P1_5
#define RIGHT_ENCODER               P6_2

#define RUN_TIME                    (20*1000)
#define SAMPLING_INTERVAL           100
#define SAMPLE_LEN                  (RUN_TIME/SAMPLING_INTERVAL)
#define WRITE                       1 // change to 1 to collect data, 0 to read collected data

#define JOLT_STEPS                  2

#define flash1 SEGMENT_D
#define flash2 SEGMENT_B

bool do_write = 1;
bool do_read = 1;

int step_num = 0;
volatile boolean do_loop = 0; // timer signal to increment timestep

int16_t deltaArr[SAMPLE_LEN] = {0};
uint16_t lpos[SAMPLE_LEN] = {0};
uint16_t rpos[SAMPLE_LEN] = {0};
uint8_t lpwm[SAMPLE_LEN] = {0};
uint8_t rpwm[SAMPLE_LEN] = {0};

typedef struct encoder {
  int pin;
  int pos;
  bool level;
  int avg;
} encoder_t;

encoder_t left_encoder = {LEFT_ENCODER, 0, LOW, 0};
encoder_t right_encoder = {RIGHT_ENCODER, 0, LOW, 0};

/*---------------------------*/
/*      CODE BLOCK CON1      */
/*     From open_loop.ino    */
/*       with changes        */
/*---------------------------*/

float theta_left = 1;
float theta_right = 1;
float beta_left = 1;
float beta_right = 1;
float v_star = 1;

// PWM inputs to jolt the car straight
int left_jolt = 1;
int right_jolt= 1;

// Control gains
float k_left = 1;
float k_right = 1;

/*---------------------------*/
/*      CODE BLOCK CON2      */
/*---------------------------*/

float driveStraight_left(float v_star, float delta) {
  return 1.0;
}

float driveStraight_right(float v_star, float delta) {
  return 1.0;
}

/*---------------------------*/
/*      CODE BLOCK CON3      */
/*---------------------------*/

float delta_ss = 0;

/*---------------------------*/
/*---------------------------*/
/*---------------------------*/

void setup(void) {
  Serial.begin(38400);

  pinMode(LEFT_MOTOR, OUTPUT);
  pinMode(LEFT_ENCODER, INPUT);
  pinMode(RIGHT_MOTOR, OUTPUT);
  pinMode(RIGHT_ENCODER, INPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);


  write_pwm(0, 0); // Turn off motors
  delay(2000); // Wait 2 seconds to put down car
  reset_blinker(); // Blink lights to indicate car is running
  setTimer(); // Set timer for timestep
}

void loop(void) {
  check_encoders();
  if (do_loop && WRITE) {
    // Apply maximum input for a short time to start motors
    if (step_num < JOLT_STEPS) {
      write_pwm(left_jolt, right_jolt);
      step_num++;
    }
    // If not done running
    else if (step_num < SAMPLE_LEN) {

      // Save positions because _left_position and _right_position
      // can change in the middle of one loop.
      int left_position = left_encoder.pos;
      int right_position = right_encoder.pos;

      /*---------------------------*/
      /*      CODE BLOCK CON0      */
      /*---------------------------*/

      /*--------------------------------------*/
      /*     Add the steady-state value of    */
      /*    delta from this calculation to    */
      /*    compensate for initial turning    */
      /*--------------------------------------*/
      float delta = left_position - right_position + delta_ss;

      // Drive straight using feedback
      // Compute the needed pwm values for each wheel using delta and v_star
      int left_cur_pwm = 1;
      int right_cur_pwm = 1;
      write_pwm(left_cur_pwm, right_cur_pwm);

      /*---------------------------*/
      /*---------------------------*/
      /*---------------------------*/

      lpos[step_num] = left_position;
      rpos[step_num] = right_position;
      deltaArr[step_num] = delta;
      lpwm[step_num] = left_cur_pwm;
      rpwm[step_num] = right_cur_pwm;

      step_num++;
    }

    else { // When step_num has reached SAMPLE_LEN
      // Turn off motors
      write_pwm(0, 0);

      if (do_write) {
        Serial.println("writing...");
        write_to_flash();
        do_write = 0;
      }
      else {
        // Print out result
        Serial.println("Start");
        Serial.println("delta - left pos - right pos - left pwm - right pwm");
        for (int i = 0; i < SAMPLE_LEN; i++) {
          Serial.print(deltaArr[i]);
          Serial.print(',');
          Serial.print(lpos[i]);
          Serial.print(',');
          Serial.print(rpos[i]);
          Serial.print(',');
          Serial.print(lpwm[i]);
          Serial.print(',');
          Serial.print(rpwm[i]);
          Serial.print('\n');
        }
      }
    }
    do_loop = 0;
  } else if (!WRITE) {
    //stop wheels
    write_pwm(0,0);
    //read and print to serial monitor
    if (do_read) {
      read_from_flash();
      do_read = 0;
    }
  }
}

/*---------------------------*/
/*     Helper functions      */
/*---------------------------*/

/*Runs after data is collected
Stores (pwm, lvel, rvel) as 3 byte chunks
Problems if:
  SAMPLE_LEN is too long (flash mem is small TwT)
  Car goes too fast - Byte cannot store speeds above 255
*/
void write_to_flash() {
  unsigned char delta;
  unsigned char ld;
  unsigned char rd;
  unsigned char rp;
  unsigned char lp;
  Flash.erase(flash1);
  Flash.erase(flash2);
  for(int i = 1; i < SAMPLE_LEN; i++) {
    delta = (unsigned char) deltaArr[i];
    ld = (unsigned char) lpos[i];
    rd = (unsigned char) rpos[i];
    rp = (unsigned char) rpwm[i];
    lp = (unsigned char) lpwm[i];
    
    if (i < SAMPLE_LEN / 2) {
      Flash.write(flash1 + 2*(i-1), &delta, 1);
      Flash.write(flash1 + 2*(i-1) + 1, &ld , 1);
      Flash.write(flash1 + 2*(i-1) + 2, &rd , 1);
      // debugging
    } else {
      Flash.write(flash2 + 2*(i - (SAMPLE_LEN/2)), &delta, 1);
      Flash.write(flash2 + 2*(i - (SAMPLE_LEN/2)) + 1, &ld, 1);
      Flash.write(flash2 + 2*(i - (SAMPLE_LEN/2)) + 2, &rd, 1);
    }
  }
}

/*Reads information gathered and put into flash
*/
void read_from_flash() {
  unsigned char delta;
  unsigned char ld;
  unsigned char rd;
  Serial.println("delta, ld, rd");
  for(int i = 1; i < SAMPLE_LEN; i++) {
    if (i< SAMPLE_LEN / 2) {
      Flash.read(flash1 + 2*(i-1), &delta, 1);
      Flash.read(flash1 + 2*(i-1) + 1, &ld, 1);
      Flash.read(flash1 + 2*(i-1) + 2, &rd, 1);
    } else {
      Flash.read(flash2 + 2*(i - (SAMPLE_LEN/2)), &delta, 1);
      Flash.read(flash2 + 2*(i - (SAMPLE_LEN/2)) + 1, &ld, 1);
      Flash.read(flash2 + 2*(i - (SAMPLE_LEN/2)) + 1, &rd, 1);
    }
    
    Serial.print( (int) delta);
    Serial.print(',');
    Serial.print((int) ld);
    Serial.print(',');
    Serial.print((int) rd);
    Serial.print('\n');    
  }

}

void write_pwm(int pwm_left, int pwm_right) {
  analogWrite(LEFT_MOTOR, (int) min(max(0, pwm_left), 255));
  analogWrite(RIGHT_MOTOR, (int) min(max(0, pwm_right), 255));
}

void reset_blinker(void) {
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

/*---------------------------*/
/*    Interrupt functions    */
/*---------------------------*/

#define AVG_DECAY_RATE              0.3
#define LOW_THRESH                  ((int) (0.2*4096))
#define HIGH_THRESH                 ((int) (0.8*4096))

void check_encoder(encoder_t* enc) {
  int new_val = analogRead(enc->pin);
  enc->avg = (int) (AVG_DECAY_RATE*enc->avg + (1 - AVG_DECAY_RATE)*new_val);
  if ((enc->level == LOW && HIGH_THRESH < enc->avg) ||
      (enc->level == HIGH && enc->avg < LOW_THRESH)) {
    enc->pos++;
    enc->level = !enc->level;
  }
}

void check_encoders(void) {
  check_encoder(&left_encoder);
  check_encoder(&right_encoder);
}

// Set timer for timestep; use A2 since A0 & A1 are used by PWM
void setTimer(void) {
  TA2CCR0 = (unsigned int) (32.768*SAMPLING_INTERVAL); // set the timer based on 32kHz clock
  TA2CCTL0 = CCIE; // enable interrupts for Timer A
  __bis_SR_register(GIE);
  TA2CTL = TASSEL_1 + MC_1 + TACLR + ID_0;
}

// ISR for timestep
#pragma vector=TIMER2_A0_VECTOR    // Timer A ISR
__interrupt void Timer2_A0_ISR(void) {
  do_loop = 1;
}
