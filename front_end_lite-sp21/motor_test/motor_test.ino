/*
 * dynamics_data.ino
 *
 * Collects dynamics data (position) with
 * some varying (bounded) input PWM
 *
 * EE16B Fall 2016
 * John Maidens, Emily Naviasky & Nathaniel Mailoa
 *
 * EE16B Fall 2017
 * Andrew Blatner
 *
 * EE16B Spring 2019
 * Mia Mirkovic
 *
 * EE16B Spring 2021
 * Hossein Najafi
 */

#define MOTOR                  9
#define RED_LED          12

#define GREEN_LED      13
#define my_delay      100
int pwm = 0;
int dir = 1;

void setup(void) {
  Serial.begin(38400);

  pinMode(MOTOR, OUTPUT);
  Serial.print("Setup done\n");

  write_pwm(); 

}

void loop(void) {

  float duty_cycle = float(pwm)/255*100;
  write_pwm();
  Serial.print("Duty cycle: ");
  Serial.print(duty_cycle,DEC);
  Serial.println("%");
  pwm = pwm + dir*5;
  if (pwm >= 255) {
    dir = -1;
  }
  if (pwm <= 0) {
    dir = 1;
  }
  delay(my_delay);
}

/*---------------------------*/
/*     Helper functions      */
/*---------------------------*/

void write_pwm(void) {
  analogWrite(MOTOR,  pwm); //(min(max(0, pwm), 255)));
}

