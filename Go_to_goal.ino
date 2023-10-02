#include<ESP32Encoder.h>
#include<CytronMotorDriver.h>
#include<cmath>

CytronMD motor2(PWM_DIR, 14, 27);
CytronMD motor1(PWM_DIR, 13, 12);


ESP32Encoder encoder1;
ESP32Encoder encoder2;

const int l1 = 10.5;
const int l2 = 10.5;

const float kp = 40;
const float ki = 0.0;
const float kd = 100;

unsigned long current_time = 0, previous_time = 0;
double elapsed_time = 0;
i
nt mapvalue(float pwm){
  if(pwm>0)
  {
    if(pwm<50) pwm=50;
     if(pwm>80) pwm=80;
    }
   else if(pwm<0){
    if(pwm>-50) pwm=-50;   
     if(pwm<80) pwm=-80;
    }
    return pwm;
  }

void drive_motor(float des_q1, float des_q2) {
  float real_q1, real_q2, current_err_q1, current_err_q2, previous_err_q1=0, previous_err_q2=0;
  float pwm_q1, pwm_q2;
  while ((real_q1 != des_q1) || (real_q2 != des_q2)) {
    
    //Reading steps taken by the encoder
	  int angle1 = encoder1.getCount();
    int angle2 = encoder2.getCount();
  
    //Converting steps to angle (We have 8000 CPR)
    angle1 = angle1 % 8000;
    angle2 = angle2 % 8000;

    //Exception for negative values
    if (angle1 < 0)angle1 += 8000;
    if (angle2 < 0)angle2 += 8000;

    //Converting steps to angles in radian
    real_q1 = angle1*(M_PI/4000);
    real_q2 = angle2*(M_PI/4000);

    //Keeping track of elapsed time
    current_time = millis();
    elapsed_time = (double)(current_time - previous_time);
    
    //Calculating current error
    current_err_q1 = des_q1 - real_q1;
    current_err_q2 = des_q2 - real_q2;

    //Converting negative error into positive error
    if(current_err_q2<0) current_err_q2+=360;
    if(current_err_q1<0) current_err_q1+=360;

    //Calculating pwm for each motor using PID
    pwm_q1 = 2*current_err_q1 + 50*(current_err_q1 - previous_err_q1)/elapsed_time;
    pwm_q2 = 40*current_err_q2 + 100*(current_err_q2 - previous_err_q2)/elapsed_time;

    //Mapping the pwm values (50,80) as the motors don't
    //run below pwm 50. And pwm above 80 causes unstability
    pwm_q1=mapvalue(pwm_q1);
    pwm_q2=mapvalue(pwm_q2);

    //Keeping track of previous error
    previous_err_q1 = current_err_q1;
    previous_err_q2 = current_err_q2;

    //Updating time
    previous_time = current_time;

    //Printing current error and pwm values for debugging
    Serial.println("Error: " + String(current_err_q1)+"      "+String(current_err_q2));
    Serial.println("PWM Values: " + String(int(pwm_q1))+"      "+String(int(pwm_q2)));

    //Driving the motor according to corresponding pwm
    motor1.setSpeed(int(pwm_q1));
    motor2.setSpeed(int(pwm_q2));
  }
}


void setup(){
	Serial.begin(115200);
	ESP32Encoder::useInternalWeakPullResistors=UP;
	encoder1.attachHalfQuad(33, 32);
	encoder2.attachHalfQuad(26, 25);
	encoder1.setCount(0);
	encoder2.clearCount();
}

void loop(){
  //Take coordinates input from user
  Serial.print("Enter x-coordinate:");
  while(Serial.available()!=0){}
  int x = Serial.parseInt();
  Serial.print("Enter y-coordinate:");
  while(Serial.available()!=0){}
  int y = Serial.parseInt();
  
  //Solve inverse kinematics
  float theta = acos(((x * x) + (y * y) - (l1 * l1) - (l2 * l2)) / (2 * l1 * l2));
  float q1 = (atan(y / x)) - (atan((l2 * sin(theta))) / ((l1) + (l2 * cos(theta))));
  float q2 = q1 + theta;

  //Drive motors to resulting angle values
  drive_motor(q1, q2);
}
