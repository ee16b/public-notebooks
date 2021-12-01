/*
 * integration.ino
 * Final sketch for SIXT33N Speech version
 *
 * EE16B Fall 2016
 * Emily Naviasky & Nathaniel Mailoa
 *
 * EE16B Fall 2017
 * Andrew Blatner
 *
 * EE16B Spring 2021
 * Zhongkai Wang
 *
 * EECS16B Fall 2021
 * Bozhi Yin
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

#define NUM_COMMANDS                4    // YOUR CODE HERE; should be less than 6
#define DRIVE_FAR                   0
#define DRIVE_LEFT                  1
#define DRIVE_CLOSE                 2
#define DRIVE_RIGHT                 3

int mode = 0; // 0 - plot mode; 1 - data collection mode
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

float theta_left = ...;
float theta_right = ...;
float beta_left = ...;
float beta_right =  ...;

// Use the following v_star value, do not need to change this
float v_star = ...;

// PWM inputs to jolt the car straight
int left_jolt = ...;
int right_jolt = ...;

// Control gains
float f_left = ...;
float f_right = ...;

/*---------------------------*/
/*      CODE BLOCK CON2      */
/*      From turning.ino     */
/*---------------------------*/
// YOUR CODE HERE

float drive_straight_left(float delta,float v_star) {
  return ...;
}

float drive_straight_right(float delta, float v_star) {
  return ...;
}

/*---------------------------*/
/*      CODE BLOCK CON3      */
/*      From turning.ino     */
/*---------------------------*/
// YOUR CODE HERE

float delta_ss = ...;

/*---------------------------*/
/*      CODE BLOCK CON4      */
/*      From turning.ino     */
/*---------------------------*/

#define CAR_WIDTH                   15.0 // in cm
#define TURN_RADIUS                 (v_star/5.0 * 80.0)  // in cm - 6 feet diameter

float delta_reference(int k) {
  // YOUR CODE HERE
  // We divide by 5 because our v_star was collected at a 5x slower sampling rate
  float ...;
  if (drive_mode == DRIVE_RIGHT) {
    return ...;
  }
  else if (drive_mode == DRIVE_LEFT) {
    return ...;
  }
  else { // DRIVE_FAR, DRIVE_CLOSE
    return ...;
  }
}

/*---------------------------*/
/*      CODE BLOCK CON5      */
/*      From turning.ino     */
/*---------------------------*/
#define INFINITY                    (3.4e+38)
#define STRAIGHT_RADIUS             INFINITY


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

// Enveloping and EUCLIDEAN_THRESHOLD constants
#define SNIPPET_SIZE                40
#define PRELENGTH                   5
#define THRESHOLD                   0.5

#define EUCLIDEAN_THRESHOLD         0.1
#define COMMANDS_SIZE				SNIPPET_SIZE * NUM_COMMANDS

/*---------------------------*/
/*---------------------------*/
/*---------------------------*/


/*---------------------------*/
/*      CODE BLOCK PCA2      */
/*     From classify.ino     */
/*---------------------------*/
// YOUR CODE HERE

float pca_vec1[SNIPPET_SIZE] = {-0.0467, -0.0533, -0.0647, -0.0773, -0.152, -0.2113, -0.3846, -0.3992, -0.3593, -0.276, -0.1782, -0.1084, -0.0423, -0.0312, -0.042, -0.0386, -0.0243, -0.0013, 0.0006, 0.0344, 0.0833, 0.1734, 0.1529, 0.1101, 0.1163, 0.1235, 0.1321, 0.1507, 0.1861, 0.1955, 0.1891, 0.1683, 0.1539, 0.1272, 0.1197, 0.1008, 0.0791, 0.0523, 0.0283, 0.013};
float pca_vec2[SNIPPET_SIZE] = {0.0059, 0.0169, 0.0545, 0.0431, -0.031, -0.2629, -0.3046, -0.2503, -0.1028, 0.0077, 0.0746, 0.1271, 0.1673, 0.2444, 0.3166, 0.3249, 0.2995, 0.2652, 0.239, 0.1908, 0.0801, -0.0751, -0.0933, -0.0606, -0.0719, -0.093, -0.0983, -0.1138, -0.1567, -0.1747, -0.1603, -0.1362, -0.1141, -0.0809, -0.0687, -0.0467, -0.025, -0.0018, 0.0271, 0.0379};
float projected_mean_vec[2] = {-0.09701674 -0.00467954};
float centroid1[2] = {-0.11839708, -0.05654983}; // DRIVE_FAR
float centroid2[2] = {0.03444015, 0.04468361}; // DRIVE_LEFT
float centroid3[2] = {-0.0317497, 0.06811907}; // DRIVE_CLOSE
float centroid4[2] = {0.11570662, -0.05625285}; // DRIVE_RIGHT
float* centroids[4] = {
  (float *) &centroid1, (float *) &centroid2,
  (float *) &centroid3, (float *) &centroid4
};

// Comment out the following line if SNIPPET_SIZE > 41
float word_array[COMMANDS_SIZE] = {0.0004, 0.0029, 0.012, 0.0156, 0.0237, 0.0683, 0.115, 0.13, 0.1268, 0.0936, 0.0952, 0.0796, 0.0591, 0.058, 0.0298, 0.0207, 0.0121, 0.0107, 0.0079, 0.0046, 0.0018, 0.0002, 0.0001, 0.0, 0.0001, 0.0001, 0.0002, 0.0076, 0.0043, 0.0002, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.004, 0.0076, 0.0062, 0.0068, 0.0149, 0.0191, 0.0256, 0.0255, 0.0348, 0.0394, 0.0351, 0.0437, 0.0389, 0.0528, 0.0576, 0.0495, 0.044, 0.044, 0.0407, 0.0403, 0.0355, 0.0361, 0.0231, 0.017, 0.0133, 0.0106, 0.0145, 0.0159, 0.0144, 0.017, 0.0174, 0.0216, 0.0176, 0.0199, 0.0169, 0.0196, 0.0161, 0.0118, 0.0127, 0.013, 0.0075, 0.0063, 0.0072, 0.0002, 0.0038, 0.0146, 0.024, 0.0296, 0.047, 0.051, 0.0518, 0.0681, 0.0621, 0.0633, 0.0698, 0.0619, 0.0653, 0.0568, 0.052, 0.054, 0.0567, 0.0549, 0.0372, 0.026, 0.0182, 0.008, 0.0058, 0.0051, 0.0037, 0.0025, 0.0016, 0.0009, 0.001, 0.0003, 0.0004, 0.0002, 0.0001, 0.0001, 0.0, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0, 0.0326, 0.0371, 0.0251, 0.0274, 0.0321, 0.0324, 0.026, 0.0232, 0.018, 0.0048, 0.0006, 0.0292, 0.0022, 0.0004, 0.0, 0.0005, 0.0694, 0.0431, 0.0191, 0.0327, 0.0422, 0.0328, 0.0294, 0.0234, 0.0375, 0.0499, 0.0581, 0.0517, 0.0454, 0.0391, 0.0376, 0.0289, 0.0262, 0.0248, 0.0108, 0.0043};

// Comment out the following lines if SNIPPET_SIZE < 42
// float projected_word0[2]=...;
// float projected_word1[2]=...;
// float projected_word2[2]=...;
// float projected_word3[2]=...;
// float* projected_word_array[NUM_COMMANDS]=...;


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
int down_sample_ratio = 1;

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
  if (mode == 1){
    down_sample_ratio = 11;
  }
  if (program_count == NUM_COMMANDS) {
      do_loop = 0;
      //program_count = 0;
      start_drive_mode();
      x_pos = 0;
      y_pos = 0;
      theta_pos = 0;
      delta = 0;
      old_delta =0;
    if (mode == 0){
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
    
    // Project 'word_array' onto the principal components
    // Comment out the following lines if SNIPPET_SIZE > 41
    proj1 = 0;
    proj2 = 0;
    for (int i = 0; i < SNIPPET_SIZE; i++) {
        proj1 += pca_vec1[i] * word_array[i+program_count*SNIPPET_SIZE];
        proj2 += pca_vec2[i] * word_array[i+program_count*SNIPPET_SIZE];
    }
    
    // Comment out the following lines if SNIPPET_SIZE < 42
    // proj1 = projected_word_array[program_count][0];
    // proj2 = projected_word_array[program_count][1];
    
    
    // YOUR CODE HERE
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

    // Compare 'best_dist' against the 'EUCLIDEAN_THRESHOLD' and print the result
    // If 'best_dist' is less than the 'EUCLIDEAN_THRESHOLD', the recording is a word
    // Otherwise, the recording is noise
    if (mode == 0){
    	if (best_dist < EUCLIDEAN_THRESHOLD) {
      		Serial.print("Classified as word:");
      		Serial.println(word_str[best_index]);
    	}
    	else {
      		Serial.print("Not classified. The closest is word: ");
      		Serial.println(word_str[best_index]);
    	}
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
      delta = delta - delta_reference(step_num-JOLT_STEPS);
      
      theta_pos = theta_pos + (delta-old_delta)/CAR_WIDTH;
      //theta_pos = theta_pos - (int)(theta_pos/6.28)*6.28;
    
      x_pos = x_pos + cos(theta_pos)*v_star;
      y_pos = y_pos + sin(theta_pos)*v_star;
      
      float rad = sqrt(x_pos*x_pos + y_pos*y_pos);
      
      if (step_num % down_sample_ratio == 0 or step_num == sample_lens){
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

    if (step_num == sample_lens) {
      // Completely stop and go back to listen MODE after 3 seconds
      delta = 0;
      if (mode == 0){
        Serial.print("Executing Next Command. \n\n");
      } else {
        Serial.print("10000");
        Serial.print(",");
        Serial.println("10000");
      }
      
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
