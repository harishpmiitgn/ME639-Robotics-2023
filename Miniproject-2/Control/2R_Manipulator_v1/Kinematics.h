/*

# Using systematic & generalizable approach
- Rotation matrices
- DH parameters
- Jacobian
- 

*/


void FK(float q1, float q2, int control=true) {
  if (control) {
    Pos_control_v1(1, q1);
    Pos_control_v1(2, q2);
  }

  q1 = q1*3.142/180;
  q2 = q2*3.142/180;
  
  float x = l1*cos(q1) + l2*cos(q2);
  float y = l1*sin(q1) + l2*sin(q2);

  Serial.print("FK Results: X = "); 
  Serial.print(x);
  Serial.print(", Y = ");
  Serial.println(y);
}

void IK_control(float x, float y, int control=true) {
  float theta = (acos((x*x + y*y - l1*l1 - l2*l2) / (2*l1*l2)))*180/3.14;
  i = (atan2(y, x)- atan2(l2*sin(theta), l1 + l2*cos(theta)))*180/3.14;
  j = i+theta;

  if (i<0) i+= 180;
  if (j<0) j+= 180;
  Serial.println(i);
  Serial.println(j);
  
  if (control) FK(i, j);
  else  delay(2000);
}

void IK_Torque_control(float x, float y, float Torque, int control=true) {
  float theta = (acos((x*x + y*y - l1*l1 - l2*l2) / (2*l1*l2)))*180/3.14;
  i = (atan2(y, x)- atan2(l2*sin(theta), l1 + l2*cos(theta)))*180/3.14;
  j = i+theta;

  if (i<0) i+= 180;
  if (j<0) j+= 180;
  Serial.print("IK Results: q1 = "); 
  Serial.print(i);
  Serial.print(", q2 = ");
  Serial.println(j);

  float Tx = Torque;
  float Ty = Torque;
  
  if (control) {
    Torque_control_v1(1, i, Tx);
    Torque_control_v1(2, j, Ty);
  }
  else  delay(2000);
}

void IK_PI_control(float x, float y, int control=true) {
  float theta = (acos((x*x + y*y - l1*l1 - l2*l2) / (2*l1*l2)))*180/3.14;
  i = (atan2(y, x)- atan2(l2*sin(theta), l1 + l2*cos(theta)))*180/3.14;
  j = i+theta;

  if (i<0) i+= 180;
  if (j<0) j+= 180;
  Serial.println(i);
  Serial.println(j);
  
  if (control) {
    Pos_PID_Control(1, i);
    Pos_PID_Control(2, j);
  }
  else  delay(2000);
}
