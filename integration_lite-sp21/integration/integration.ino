/*
 * integration.ino
 * Final sketch for SIXT33N Speech version
 *
 * EE16B Fall 2016
 * Emily Naviasky & Nathaniel Mailoa
 *
 * EE 16B Fall 2017
 * Andrew Blatner
 *
 * EE16B Spring 2021
 * Zhongkai Wang
 *
 */

/********************************************************/
/********************************************************/
/***                                                  ***/
/*** Constants and global variables from turning.ino. ***/
/***                                                  ***/
/********************************************************/
/********************************************************/

#define LEFT_MOTOR    10   // Input L
#define RIGHT_MOTOR   9    // Input R
#define encoderPinL   3    // Encoder L
#define encoderPinR   2    // Encoder R

#define RUN_TIME                    (20*1000)
#define SAMPLING_INTERVAL           100
#define SAMPLE_INIT                 10

#define SAMPLE_LEN                  50+SAMPLE_INIT //(RUN_TIME/SAMPLING_INTERVAL)
#define JOLT_STEPS                  0


// Operation modes
#define MODE_LISTEN                 0
#define MODE_DRIVE                  1

#define NUM_COMMANDS                4    // YOUR CODE HERE
#define DRIVE_FAR                   0
#define DRIVE_LEFT                  1
#define DRIVE_CLOSE                 2
#define DRIVE_RIGHT                 3


boolean loop_mode = MODE_DRIVE;
int drive_mode;
int program_count;

int step_num = 0;
volatile boolean do_loop = 1;       // timer signal to increment timestep
volatile long encoderValueL = 0;
volatile long encoderValueR = 0;

float x_pos = 0;
float y_pos = 0;
float theta_pos = 0;
float old_delta = 0;
float delta = 0;

typedef struct encoder {
  int pin;
  int pos;
  bool level;
  int avg;
} encoder_t;

encoder_t left_encoder = {encoderPinL, 0, LOW, 0};
encoder_t right_encoder = {encoderPinR, 0, LOW, 0};

/*---------------------------*/
/*      CODE BLOCK CON1      */
/*      From turning.ino     */
/*---------------------------*/
// YOUR CODE HERE

float theta_left = ;
float theta_right = ;
float beta_left = ;
float beta_right = ;

// Use the following v_star value, do not need to change this
float v_star = 201.759;

// PWM inputs to jolt the car straight
int left_jolt = ;
int right_jolt = ;

// Control gains
float k_left = ;
float k_right = ;

/*---------------------------*/
/*      CODE BLOCK CON2      */
/*      From turning.ino     */
/*---------------------------*/
// YOUR CODE HERE

float driveStraight_left(float delta,float v_star) {
  return ;
}

float driveStraight_right(float delta, float v_star) {
  return ;
}

/*---------------------------*/
/*      CODE BLOCK CON3      */
/*      From turning.ino     */
/*---------------------------*/
// YOUR CODE HERE

float delta_ss = ;

/*---------------------------*/
/*      CODE BLOCK CON4      */
/*      From turning.ino     */
/*---------------------------*/

#define CAR_WIDTH                   15.0 // in cm
#define TURN_RADIUS                 (v_star/5.0 * 80.0)  // in cm - 6 feet diameter

/*---------------------------*/
/*    PREPROGRAMMED PATH     */
/*---------------------------*/

float delta_reference(int k) {
  // YOUR CODE HERE
  // We divide by 5 because our v_star was collected at a 5x slower sampling rate
  float delta = ;
  if (drive_mode == DRIVE_RIGHT) {
    return ;
  }
  else if (drive_mode == DRIVE_LEFT) {
    return ;
  }
  else { // DRIVE_FAR, DRIVE_CLOSE
    return ;
  }
}

/*---------------------------*/
/*      CODE BLOCK CON5      */
/*      From turning.ino     */
/*---------------------------*/
#define INFINITY                    (3.4e+38)
#define STRAIGHT_RADIUS             INFINITY
// #define STRAIGHT_RADIUS             5000

float straight_correction(int k) {
  // YOUR CODE HERE
  return (CAR_WIDTH * v_star/5 * k)/STRAIGHT_RADIUS;
}

/*---------------------------*/
/*---------------------------*/
/*---------------------------*/

/*********************************************************/
/*********************************************************/
/***                                                   ***/
/*** Constants and glboal variables from classify.ino. ***/
/***                                                   ***/
/*********************************************************/
/*********************************************************/

#define SIZE                        200
#define SIZE_AFTER_FILTER           1

/*---------------------------*/
/*      CODE BLOCK PCA1      */
/*     From classify.ino     */
/*---------------------------*/
// YOUR CODE HERE

// Enveloping and K-means constants
#define SNIPPET_SIZE                40
#define PRELENGTH                   5
#define THRESHOLD                   0.5

#define KMEANS_THRESHOLD            0.1
#define LOUDNESS_THRESHOLD          100

/*---------------------------*/
/*---------------------------*/
/*---------------------------*/


/*---------------------------*/
/*      CODE BLOCK PCA2      */
/*     From classify.ino     */
/*---------------------------*/
// YOUR CODE HERE

float pca_vec1[SNIPPET_SIZE] = ;
float pca_vec2[SNIPPET_SIZE] = ;
float projected_mean_vec[2] = ;
float centroid1[2] = ;   // DRIVE_FAR
float centroid2[2] = ;   // DRIVE_LEFT
float centroid3[2] = ;   // DRIVE_CLOSE
float centroid4[2] = ;   // DRIVE_RIGHT
float* centroids[4] = {
  (float *) &centroid1, (float *) &centroid2,
  (float *) &centroid3, (float *) &centroid4
};

// Word 1 sample
float word1[SNIPPET_SIZE] = ;
// Word 2 sample
float word2[SNIPPET_SIZE] = ;
// Word 3 sample
float word3[SNIPPET_SIZE] = ;
// Word 4 sample
float word4[SNIPPET_SIZE] = ;

/*---------------------------*/
/*      CODE BLOCK DRV       */
/*---------------------------*/
// YOUR CODE HERE
// Change word_arr for different trajectory

float* word_arr[NUM_COMMANDS] = {(float *) &word2, (float *) &word1, (float *) &word4, (float *) &word3};

/*---------------------------*/
/*---------------------------*/
/*---------------------------*/

float proj1 = 0;
float proj2 = 0;

char word1_str[] = "Drive Far";
char word2_str[] = "Drive Left";
char word3_str[] = "Drive Close";
char word4_str[] = "Drive Right";
char* word_str[] = {(char *) &word1_str, (char *) &word2_str, (char *) &word3_str, (char *) &word4_str};

float x_pos_arr[NUM_COMMANDS+1] = {0};
float y_pos_arr[NUM_COMMANDS+1] = {0};
int word_idx[NUM_COMMANDS] = {0};

/*---------------------------*/
/*       Norm functions      */
/*---------------------------*/

// Compute the L2 norm of (dim1, dim2) and centroid
// input: dim1: 1st dimension coordinate
//        dim2: 2nd dimension coordinate
//        centroid: size-2 array containing centroid coordinates
// output: L2 norm (Euclidean distance) between point and centroid
float l2_norm(float dim1, float dim2, float* centroid) {
  return sqrt(pow(dim1-centroid[0],2) + pow(dim2-centroid[1],2));
}

// Compute the L2 norm of (dim1, dim2, dim3) and centroid
// input: dim1: 1st dimension coordinate
//        dim2: 2nd dimension coordinate
//        dim3: 3rd dimension coordinate
//        centroid: size-3 array containing centroid coordinates
// output: L2 norm (Euclidean distance) between point and centroid
float l2_norm3(float dim1, float dim2, float dim3, float* centroid) {
  return sqrt(pow(dim1-centroid[0],2) + pow(dim2-centroid[1],2) + pow(dim3-centroid[2],2));
}

int sample_lens = 12600;

void setup(void) {
  Serial.begin(38400);

  pinMode(encoderPinL, INPUT_PULLUP); 
  pinMode(encoderPinR, INPUT_PULLUP);
  pinMode(LEFT_MOTOR, OUTPUT);
  pinMode(RIGHT_MOTOR, OUTPUT);

  // for (int i = 0; i < NUM_COMMANDS; i++) {
  //  sample_lens[i] = run_times[i] / SAMPLING_INTERVAL;
  // }

  write_pwm(0, 0);
  attachInterrupt(digitalPinToInterrupt(encoderPinL), updateEncoderL,CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinR), updateEncoderR,CHANGE);
  
  write_pwm(0, 0); // Turn off motors
  start_listen_mode();
  do_loop = 1;
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
      Serial.println("Done.");
      
      Serial.println("Positions after each command.");
      for(int j = 0; j < NUM_COMMANDS+1; j++){
        Serial.print(x_pos_arr[j]);
        Serial.print(",");
        Serial.println(y_pos_arr[j]);
      }
      
      Serial.println("The commands are:");
      for(int j = 0; j < NUM_COMMANDS; j++){
        Serial.println(word_str[word_idx[j]]);
      }
    
      program_count++;
  }
  else if (loop_mode == MODE_LISTEN) {
    // In the integration phase of the project, this section will listen
    // to the microphone and switch to the specified mode.
    // For now, we simply cycle through them.
    /*---------------------------*/
    /*      CODE BLOCK PCA3      */
    /*---------------------------*/
    proj1 = 0;
    proj2 = 0;
    // Project 'word_arr[program_count]' onto the principal components
    // Hint: each 'word' is an array
    // Hint: do this entire operation in 1 loop by replacing the '...'
    // YOUR CODE HERE
    for (int i = 0; i < SNIPPET_SIZE; i++) {
      proj1 += ...;
      proj2 += ...;
    }

    // Demean the projection
    proj1 -= ...;
    proj2 -= ...;

    // Classification
    // Use the function 'l2_norm' defined above
    // ith centroid: 'centroids[i]'
    float best_dist = 999999;
    int best_index = -1;
    // YOUR CODE HERE: set 'best_dist' and 'best_index' to appropriate values.
    ...

    // Compare 'best_dist' against the 'KMEANS_THRESHOLD' and print the result
    // If 'best_dist' is less than the 'KMEANS_THRESHOLD', the recording is a word
    // Otherwise, the recording is noise
    if (best_dist < KMEANS_THRESHOLD) {
      Serial.print("Classified as word:");
      Serial.println(word_str[best_index]);
    }
    else {
      Serial.print("Not classified. The closest is word: ");
      Serial.println(word_str[best_index]);
    }
        
    /*---------------------------*/
    /*---------------------------*/
    /*---------------------------*/
    drive_mode = best_index;    
    // drive_mode = drive_modes[program_count];
    start_drive_mode();
    // Serial.print(program_count);
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
      //theta_pos = theta_pos - (int)(theta_pos/6.28)*6.28;
    
      x_pos = x_pos + cos(theta_pos)*v_star;
      y_pos = y_pos + sin(theta_pos)*v_star;
      
      float rad = sqrt(x_pos*x_pos + y_pos*y_pos);
      
      Serial.print(x_pos/TURN_RADIUS);
      Serial.print(",");
      Serial.println(y_pos/TURN_RADIUS);
      // Drive straight using feedback
      // Compute the needed pwm values for each wheel using delta and v_star
      int left_cur_pwm = driveStraight_left(delta, v_star);
      int right_cur_pwm = driveStraight_right(delta, v_star);
      write_pwm(left_cur_pwm, right_cur_pwm);

      /*---------------------------*/
      /*---------------------------*/
      /*---------------------------*/
    }

    // Counter for how many times loop is executed since entering DRIVE MODE
    step_num++;

    if (step_num == sample_lens) {
      // Completely stop and go back to listen MODE after 3 seconds
      delta = 0;
      Serial.print("Executing Next Command. \n\n");
      program_count++;
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
  if (drive_mode == DRIVE_CLOSE){
    sample_lens = 6300/SAMPLING_INTERVAL;
  }
  else {
    sample_lens = 12600/SAMPLING_INTERVAL;
  }
  word_idx[program_count] = drive_mode;  
}

void start_listen_mode(void) {
  write_pwm(0, 0);
  delay(300);
  loop_mode = MODE_LISTEN;
  x_pos_arr[program_count] = x_pos/TURN_RADIUS;
  y_pos_arr[program_count] = y_pos/TURN_RADIUS;
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
