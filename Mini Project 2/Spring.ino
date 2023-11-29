#include<CytronMotorDriver.h>
#include<cmath>
#include<Encoder.h>

CytronMD motor1(PWM_DIR, 3, 4);
CytronMD motor2(PWM_DIR, 9, 10);

Encoder encoder(1, 2);

const int l1 = 5;
const int l2 = 3;

const float kp = 1.0;
const float ki = 0.0;
const float kd = 0.0;

unsigned long current_time=0, previous_time=0;
double elapsed_time=0;

void PID_solver(float current_err_q1, float current_err_q2, float previous_err_q1, float previous_err_q2, int* pwm_q1, int* pwm_q2){
  *pwm_q1 = (kp*current_err_q1)+(kd*((current_err_q1 - previous_err_q1)/elapsed_time));
  *pwm_q2 = (kp*current_err_q2)+(kd*((current_err_q2 - previous_err_q2)/elapsed_time));
}

void read_angles(float* real_q1, float* real_q2){
  long encoderPosition = encoder.read();
  // Calculate the angle based on the encoder's PPR (Pulses Per Revolution)
  // Assuming 360 PPR for this example, you can adjust this value based on your encoder's specification.
  float angle = (encoderPosition * 360.0) / 360.0;

}

void drive_motor(float des_q1, float des_q2){
  float real_q1, real_q2, current_err_q1, current_err_q2, previous_err_q1, previous_err_q2;
  int pwm_q1, pwm_q2;
  while((real_q1!=des_q1)||(real_q2!=des_q2)){
    read_angles(&real_q1, &real_q2);
    
    current_time = millis();
    elapsed_time = (double)(current_time - previous_time);
    current_err_q1 = des_q1 - real_q1;
    current_err_q2 = des_q2 - real_q2;
    
    PID_solver(current_err_q1, current_err_q2, previous_err_q1, previous_err_q2, &pwm_q1, &pwm_q2);
    
    previous_err_q1 = current_err_q1;
    previous_err_q2 = current_err_q2;
    previous_time = current_time;
    
    motor1.setSpeed(pwm_q1);
    motor2.setSpeed(pwm_q2);
  }
}

void IK_solver(int x, int y, float* q1, float* q2){
  *q2 = acos(((x*x)+(y*y)-(l1*l1)-(l2*l2))/(2*l1*l2));
  *q1 = (atan(y/x)) - (atan((l2*sin(*q2)))/((l1)+(l2*cos(*q2))));
}

void setup() {
  Serial.begin(9600);
  // Initiate all pins here
}

void loop() {
 	float q1=0, q2=0;
   int x_coord, y_coord;
  while(true){
    Serial.println("Please enter the x-coordinate: ");
    while(Serial.available()>0){
      x_coord = Serial.parseInt();
    }
    Serial.println("Please enter the y-coordinate: ");
    while(Serial.available()>0){
      y_coord = Serial.parseInt();            
    }
    IK_solver(x_coord, y_coord, &q1, &q2);
    drive_motor(q1, q2);  
  }

  drive_motor(q1,q2);
}
