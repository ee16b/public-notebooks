/*
 * closed_loop.ino
 *
 * EE16B Fall 2016
 * John Maidens, Emily Naviasky & Nathaniel Mailoa
 *
 * EE16B Fall 2017
 * Andrew Blatner
 *
 * EE16B Spring 2021
 * Hossein Najafi
 */

#define LEFT_MOTOR  10  // Input L
#define RIGHT_MOTOR   9   // Input R
#define encoderPinL   3 // Encoder L
#define encoderPinR   2 // Encoder R


#define RUN_TIME                    (20*1000)
#define SAMPLING_INTERVAL           200
#define SAMPLE_INIT         10

#define SAMPLE_LEN                 50+SAMPLE_INIT //(RUN_TIME/SAMPLING_INTERVAL)


#define JOLT_STEPS                  10

int step_num = 0;
volatile boolean do_loop = 1; // timer signal to increment timestep
volatile long encoderValueL = 0;
volatile long encoderValueR = 0;

int16_t deltaArr[SAMPLE_LEN] = {0};
uint16_t lpos[SAMPLE_LEN] = {0};
uint16_t rpos[SAMPLE_LEN] = {0};
uint8_t lpwm[SAMPLE_LEN] = {0};
uint8_t rpwm[SAMPLE_LEN] = {0};

typedef struct encoder{
  int pin;
  int pos;
  bool level;
  int avg;
} encoder_t;

encoder_t left_encoder = {encoderPinL, 0, LOW, 0};
encoder_t right_encoder = {encoderPinR, 0, LOW, 0};

/*---------------------------*/
/*      CODE BLOCK CON1      */
/*     From open_loop.ino    */
/*       with changes        */
/*---------------------------*/

float theta_left = ;
float theta_right = ;
float beta_left =;
float beta_right = ;
float v_star = ;

// PWM inputs to jolt the car straight
int left_jolt = ;
int right_jolt = ;

// Control gains
float k_left = ;
float k_right = ;

/*---------------------------*/
/*      CODE BLOCK CON2      */
/*---------------------------*/

float driveStraight_left(float v_star, float delta) {
  return ;
}

float driveStraight_right(float v_star, float delta) {
  return ;
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

  pinMode(encoderPinL, INPUT_PULLUP); 
  pinMode(encoderPinR, INPUT_PULLUP);
  
  pinMode(LEFT_MOTOR, OUTPUT);
  
  pinMode(RIGHT_MOTOR, OUTPUT);
  

  write_pwm(0, 0); // Turn off motors
  attachInterrupt(digitalPinToInterrupt(encoderPinL), updateEncoderL,CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinR), updateEncoderR,CHANGE);

}

void loop(void) {
  check_encoders();
  if (do_loop) {
    // Apply maximum input for a short time to start motors
    if (step_num < JOLT_STEPS) {
      write_pwm(left_jolt, right_jolt);
      step_num++;
    }
    // If not done running
    else if (step_num < SAMPLE_LEN) {

      // Save positions because _left_position and _right_position
      // can change in the middle of one loop.
      int left_position = encoderValueL; //left_encoder.pos;
      int right_position = encoderValueR; //right_encoder.pos;

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
      int left_cur_pwm = ;
      int right_cur_pwm = ;
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

      // Print out result
      Serial.println("Start");
      Serial.println("delta - left pos - right pos - left pwm - right pwm");
      for (int i = SAMPLE_INIT; i < SAMPLE_LEN; i++) {
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
      do_loop = 0;
    }
    //do_loop = 1;
  }
  encoderValueL = 0;
  encoderValueR = 0;
  delay(SAMPLING_INTERVAL);       
}

/*---------------------------*/
/*     Helper functions      */
/*---------------------------*/

void write_pwm(int pwm_left, int pwm_right) {
  analogWrite(LEFT_MOTOR, (int) min(max(0, pwm_left), 255));
  analogWrite(RIGHT_MOTOR, (int) min(max(0, pwm_right), 255));
}

/*---------------------------*/
/*    Interrupt functions    */
/*---------------------------*/

#define AVG_DECAY_RATE              0.3
#define LOW_THRESH                  ((int) (0.2*4096))
#define HIGH_THRESH                 ((int) (0.8*4096))


void check_encoders(void) {
  encoder_t *enc = &left_encoder;
  enc->pos = encoderValueL;
  enc = &right_encoder;
  enc->pos = encoderValueR;
  
}


/*---------------------------*/
/*        Interruptions      */
/*---------------------------*/

void updateEncoderL()
{
  encoderValueL++;
  //Serial.print(encoderValueL);
  
}
void updateEncoderR()
{
  encoderValueR++;
  //Serial.print(encoderValueR);
}