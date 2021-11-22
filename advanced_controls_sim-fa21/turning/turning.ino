/*
 * turning.ino
 *
 * Implementing turns in SIXT33N
 *
 * EE16B Fall 2016
 * Emily Naviasky & Nathaniel Mailoa
 *
 * EE 16B Fall 2017
 * Andrew Blatner
 * EE16B Spring 2021 
 * Hossein Najafi
 * EECS16B Fall 2021 
 * Yi-Hsuan Shih
 */

#define LEFT_MOTOR  10  // Input L
#define RIGHT_MOTOR   9   // Input R
#define encoderPinL   3 // Encoder L
#define encoderPinR   2 // Encoder R


#define RUN_TIME                    (20*1000)
#define SAMPLING_INTERVAL           100
#define SAMPLE_INIT         10

#define SAMPLE_LEN                 50+SAMPLE_INIT //(RUN_TIME/SAMPLING_INTERVAL)


#define JOLT_STEPS                  10
#define SAMPLE_CONST				10 // sampling constant for plotting trajectory in python notebook

// Operation modes
#define MODE_LISTEN                 0
#define MODE_DRIVE                  1

#define NUM_COMMANDS                4
#define DRIVE_STRAIGHT              0
#define DRIVE_LEFT                  1
#define DRIVE_RIGHT                 2
#define STOP                        3

int mode = 0;
boolean loop_mode = MODE_DRIVE;
int drive_mode;
int program_count;
int sample_lens[NUM_COMMANDS] = {0};
int sample_output;

int step_num = 0;
volatile boolean do_loop = 1; // timer signal to increment timestep
volatile long encoderValueL = 0;
volatile long encoderValueR = 0;

float x_pos = 0;
float y_pos = 0;
float theta_pos = 0;
float old_delta = 0;
float delta = 0;
      
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
/*    From closed_loop.ino   */
/*---------------------------*/

float theta_left = 1.758;
float theta_right = 1.509;
float beta_left = -3.996;
float beta_right =  -3.392;

// Use the following v_star value, do not need to change this
float v_star = 202.343;

// PWM inputs to jolt the car straight
int left_jolt = 200;
int right_jolt = 240;

// Control gains
float f_left = 0.2;
float f_right = 0.2;

/*---------------------------*/
/*      CODE BLOCK CON2      */
/*    From closed_loop.ino   */
/*---------------------------*/

float drive_straight_left(float delta,float v_star) {
  return ;
}

float drive_straight_right(float delta, float v_star) {
  return ;
}

/*---------------------------*/
/*      CODE BLOCK CON3      */
/*    From closed_loop.ino   */
/*---------------------------*/

float delta_ss = ;

/*---------------------------*/
/*      CODE BLOCK CON4      */
/*---------------------------*/

#define CAR_WIDTH                   15.0 // in cm
#define TURN_RADIUS                 (v_star/5.0 * 80.0) // in cm - 6 feet diameter

/*---------------------------*/
/*    PREPROGRAMMED PATH     */
/*---------------------------*/
int run_times[NUM_COMMANDS] = {8000, 12600, 8000, 12600}; // length of commands roughly in ms
int drive_modes[NUM_COMMANDS] = {DRIVE_STRAIGHT, DRIVE_LEFT, DRIVE_STRAIGHT, DRIVE_RIGHT}; // commands: [DRIVE_STRAIGHT, DRIVE_LEFT, DRIVE_RIGHT]

float delta_reference(int n) {
  // YOUR CODE HERE
  // We divide by 5 because our v_star was collected at a 5x slower sampling rate
  float delta = (CAR_WIDTH * v_star/5.0 * n)/TURN_RADIUS;
  if (drive_mode == DRIVE_RIGHT) {
    return ;
  }
  else if (drive_mode == DRIVE_LEFT) {
    return ;
  }
  else { // DRIVE_STRAIGHT
    return ;
  }
}

/*---------------------------*/
/*      CODE BLOCK CON5      */
/*---------------------------*/
#define INFINITY                    (3.4e+38)
#define STRAIGHT_RADIUS             INFINITY

float straight_correction(int n) {
  return 0;
}

/*---------------------------*/
/*---------------------------*/
/*---------------------------*/

void setup(void) {
  Serial.begin(38400);

  pinMode(encoderPinL, INPUT_PULLUP); 
  pinMode(encoderPinR, INPUT_PULLUP);
  
  
  pinMode(LEFT_MOTOR, OUTPUT);
  pinMode(RIGHT_MOTOR, OUTPUT);
  
  
  for (int i = 0; i < NUM_COMMANDS; i++) {
    sample_lens[i] = run_times[i] / SAMPLING_INTERVAL + JOLT_STEPS;
  }

  write_pwm(0, 0);
  attachInterrupt(digitalPinToInterrupt(encoderPinL), updateEncoderL,CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinR), updateEncoderR,CHANGE);
  
  program_count = 0;
  do_loop = 1;
  start_drive_mode();
}


void loop(void) {
  check_encoders();
  if (program_count == NUM_COMMANDS) {
      do_loop = 0;
      //program_count = 0;
      start_drive_mode();
      x_pos = 0;
      y_pos = 0;
      theta_pos = 0;
      delta = 0;
      old_delta =0;
  }
  else if (loop_mode == MODE_LISTEN) {
    // In the integration phase of the project, this section will listen
    // to the microphone and switch to the specified mode.
    // For now, we simply cycle through them.
    program_count++;
    drive_mode = drive_modes[program_count];
    
    start_drive_mode();
    
  }

  else if (loop_mode == MODE_DRIVE && do_loop) {
    if (step_num < JOLT_STEPS) {
      write_pwm(left_jolt, right_jolt);
    }
    else {
    
      // Save positions because _left_position and _right_position
      // can change in the middle of one loop.
      int left_position = encoderValueL;
      int right_position = encoderValueR;

      /*---------------------------*/
      /*      CODE BLOCK CON0      */
      /*---------------------------*/
     old_delta = delta;
      
      delta = left_position - right_position + delta_ss;
      delta = delta - delta_reference(step_num-JOLT_STEPS) - straight_correction(step_num-JOLT_STEPS);
      
      theta_pos = theta_pos + (delta-old_delta)/CAR_WIDTH;
    
      x_pos = x_pos + cos(theta_pos)*v_star;
      y_pos = y_pos + sin(theta_pos)*v_star;
      
      float rad = sqrt(x_pos*x_pos + y_pos*y_pos);
        
      if (mode == 0){
      	sample_output = 1;
      }
      else{
        sample_output = SAMPLE_CONST;
      }
      if (step_num % sample_output ==0 or step_num == sample_lens[program_count]){
        Serial.print(x_pos/TURN_RADIUS);
        Serial.print(",");
        Serial.println(y_pos/TURN_RADIUS);
      }
      // Drive straight using feedback
      // Compute the needed pwm values for each wheel using delta and v_star
      int left_cur_pwm = drive_straight_left(delta, v_star);
      int right_cur_pwm = drive_straight_right(delta, v_star);
      write_pwm(left_cur_pwm, right_cur_pwm);

      /*---------------------------*/
      /*---------------------------*/
      /*---------------------------*/
    }

    // Counter for how many times loop is executed since entering DRIVE MODE
    step_num++;

    if (step_num == sample_lens[program_count]) {
      // Completely stop and go back to listen MODE after 3 seconds
      delta = 0;
      
      if (mode == 0){
      	Serial.println("Executing Next Command");
      }
      else{
        Serial.print("10000");
        Serial.print(",");
        Serial.println("10000");
      }
      start_listen_mode();
    }

  }
}

/*---------------------------*/
/*     Helper functions      */
/*---------------------------*/

void start_drive_mode(void) {
  loop_mode = MODE_DRIVE;
  step_num = 0;
  left_encoder.pos = 0;
  right_encoder.pos = 0;
}

void start_listen_mode(void) {
  write_pwm(0, 0);
  delay(300);
  loop_mode = MODE_LISTEN;
}

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
