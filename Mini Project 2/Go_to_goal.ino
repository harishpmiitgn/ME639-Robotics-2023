#include <CytronMotorDriver.h>
#include <cmath>
#include <ESP32Encoder.h>



ESP32Encoder encoder1;
// ESP32Encoder encoder2;

const int l1 = 10.5;
const int l2 = 10.5;

const float kp = 0.69420;
const float ki = 0.0;
const float kd = 0;

unsigned long current_time = 0, previous_time = 0;
double elapsed_time = 0;

// void PID_solver(float current_err_q1, float current_err_q2, float previous_err_q1, float previous_err_q2, int* pwm_q1, int* pwm_q2) {
//   *pwm_q1 = (kp * current_err_q1) + (kd * ((current_err_q1 - previous_err_q1) / elapsed_time));
//   *pwm_q2 = (kp * current_err_q2) + (kd * ((current_err_q2 - previous_err_q2) / elapsed_time));
// }

// void read_angles(float* real_q1, float* real_q2) {
//   int angle1 = ((int32_t)encoder1.getCount() * 360) / 8000;
//   int angle2 = ((int32_t)encoder2.getCount() * 360) / 8000;
//   angle1 = angle1 % 360;
//   if (angle1 < 0)
//     angle1 += 360;
//   angle2 = angle2 % 360;
//   if (angle2 < 0)
//     angle2 += 360;
//   *real_q1 = angle1;
//   *real_q2 = angle2;
// }

// void drive_motor(float des_q1, float des_q2) {
//   float real_q1, real_q2, current_err_q1, current_err_q2, previous_err_q1=0, previous_err_q2=0;
//   int pwm_q1, pwm_q2;
//   while ((real_q1 != des_q1) || (real_q2 != des_q2)) {
//     read_angles(&real_q1, &real_q2);

//     current_time = millis();
//     elapsed_time = (double)(current_time - previous_time);
//     current_err_q1 = des_q1 - real_q1;
//     current_err_q2 = des_q2 - real_q2;

//   pwm_q1 = (kp * current_err_q1) + (kd * ((current_err_q1 - previous_err_q1) / elapsed_time));
//   pwm_q2 = (kp * current_err_q2) + (kd * ((current_err_q2 - previous_err_q2) / elapsed_time));

//     previous_err_q1 = current_err_q1;
//     previous_err_q2 = current_err_q2;
//     previous_time = current_time;

//     motor1.setSpeed(pwm_q1);
//     motor2.setSpeed(pwm_q2);
//   }
// }


// void IK_solver(int x, int y, float* q1, float* q2) {
//   *q2 = acos(((x * x) + (y * y) - (l1 * l1) - (l2 * l2)) / (2 * l1 * l2));
//   *q1 = (atan(y / x)) - (atan((l2 * sin(*q2))) / ((l1) + (l2 * cos(*q2))));
// }

void setup() {
  Serial.begin(115200);
  Serial.print("I am nigger gay");
  // // Initiate all pins here
  // ESP32Encoder::useInternalWeakPullResistors=UP;

  // encoder1.attachHalfQuad(33,32);
  // encoder1.setCount(0);
  // encoder2.attachHalfQuad(26,25);
  // encoder2.setCount(0);
  delay(2000);
}

void loop() {
  float q1 = 0, q2 = 0;
  int x=1, y=1;
  q2 = acos(((x * x) + (y * y) - (l1 * l1) - (l2 * l2)) / (2 * l1 * l2));
  q1 = (atan(y / x)) - (atan((l2 * sin(q2))) / ((l1) + (l2 * cos(q2))));
  
  Serial.println(q1);
  Serial.println(q2);


// //DRIVEMOTOR//


//   float real_q1, real_q2, current_err_q1, current_err_q2, previous_err_q1=0, previous_err_q2=0;
//   int pwm_q1, pwm_q2;
//   while ((real_q1 != q1) || (real_q2 != q2)) {

// //ANGLEREAD//


//   int angle1 = ((int32_t)encoder1.getCount() * 360) / 8000;
//   int angle2 = ((int32_t)encoder2.getCount() * 360) / 8000;
//   angle1 = angle1 % 360;
//   if (angle1 < 0)
//     angle1 += 360;
//   angle2 = angle2 % 360;
//   if (angle2 < 0)
//     angle2 += 360;
//   real_q1 = angle1*(6.28/360);
//   real_q2 = angle2*(6.28/360);
// ////////////////////////////////////////////////
//     current_time = millis();
//     elapsed_time = (double)(current_time - previous_time);
//     current_err_q1 = q1 - real_q1;
//     current_err_q2 = q2 - real_q2;

//   pwm_q1 = (kp * current_err_q1) + (kd * ((current_err_q1 - previous_err_q1) / elapsed_time));
//   pwm_q2 = (kp * current_err_q2) + (kd * ((current_err_q2 - previous_err_q2) / elapsed_time));

//     previous_err_q1 = current_err_q1;
//     previous_err_q2 = current_err_q2;
//     previous_time = current_time;

//     motor1.setSpeed(pwm_q1);
//     motor2.setSpeed(pwm_q2);
//   }
}
