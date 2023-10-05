
// PID Parameter
float e = 0;    // error between the desired output and the reading

double sensed_output, control_signal;
double setpoint;
double Kp; //proportional gain
double Ki; //integral gain
double Kd; //derivative gain
int T; //sample time in milliseconds (ms)
unsigned long last_time;
double total_error, last_error;
int max_control = 50;
int min_control = 25;

int pwm = 50;

void Pos_control_v1(int link, float q) {
  float mot, dir, pos;
  if (link == 1) {
    mot = ch1;
    dir = M1_dir;
    pos = M1_encoder();
  } else if (link == 2) {
    mot = ch2;
    dir = M2_dir;
    pos = M2_encoder();
  } else return;
  
  Serial.print("Link-");
  Serial.print(link);
  Serial.print(" Position: ");
  Serial.println(pos);

  e = abs(q-pos);

  if (e > 30) {
    ledcWrite(mot, 60);  
  } else if (e > 5) {
    ledcWrite(mot, e+30);  
  } else {
    ledcWrite(mot, 0);  
  }
  if (q>pos) {
    digitalWrite(dir, LOW); 
    //Serial.println("Increse...");
  } else if (q<pos) {
    digitalWrite(dir, HIGH);
    //Serial.println("Decrese...");
  } else {
    ledcWrite(mot, 0);   
  }

  //pwm -= 0.01;
  Serial.println(e);
}

void Pos_PID_Control(int link, float q) {
  float mot, dir, pos;
  if (link == 1) {
    mot = ch1;
    dir = M1_dir;
    pos = M1_encoder();
  } else if (link == 2) {
    mot = ch2;
    dir = M2_dir;
    pos = M2_encoder();
  } else return;
  
  Serial.print("Link-");
  Serial.print(link);
  Serial.print(" Position: ");
  Serial.println(pos);

  unsigned long current_time = millis(); 
  int delta_time = current_time - last_time; //delta time interval 
  
  if (delta_time >= T){
    double error = q - pos;
    
    total_error += error; //accumalates the error - integral term
    if (total_error >= max_control) total_error = max_control;
    else if (total_error <= min_control) total_error = min_control;
    
    double delta_error = error - last_error; //difference of error for derivative term
    
    control_signal = Kp*error + (Ki*T)*total_error + (Kd/T)*delta_error; //PID control compute
    if (control_signal >= max_control) control_signal = max_control;
    else if (control_signal <= min_control) control_signal = min_control;
    
    last_error = error;
    last_time = current_time;
  } 
}
// Bound the input value between x_min and x_max.
float bound(float x, float x_min, float x_max) {
    if (x < x_min) { x = x_min; }
    if (x > x_max) { x = x_max; }
    return x;
}
