int x_pos = 0, y_pos = 0, k = 0;
int Trajectory[][2] = {
  {20, 0},
  {10, 10},
  {0, 20},
  {-10, 10},
  {-20, 0},
  {-10, -10},
  {0, -20},
  {10, -10},
  {0, -20},
  {-10, -10},
  {-20, 0},
  {-10, 10},
  {0, 20},
  {0, 0},
  {10, 10},
};


void task_1(float x_pos, float y_pos) {
  // Position control

  Serial.print("Trajectory: "); 
  Serial.print(x_pos); 
  Serial.print(", "); 
  Serial.print(y_pos); 
  Serial.println(); 

  //FK(i, j);   // Angle based control
  IK_control(x_pos, y_pos, true);    // X, Y position based control
}

void task_2() {
  // Trajectory control

  Serial.print("Trajectory: "); 
  Serial.print(x_pos); 
  Serial.print(", "); 
  Serial.print(y_pos); 
  Serial.println(); 

  //FK(i, j);
  IK_control(x_pos, y_pos, true);    // X, Y position

  if (2000+t < millis()) {
    t = millis();
    
    x_pos = Trajectory[k][0];
    y_pos = Trajectory[k][1];
    k += 1;

    if (k > 8) k=0;  
  }
}

void task_3(float x_pos, float y_pos, float Torque) {
  // Torque control

  Serial.print("Trajectory: "); 
  Serial.print(x_pos); 
  Serial.print(", "); 
  Serial.print(y_pos); 
  Serial.println(); 

  //FK(i, j);
  IK_control(x_pos, y_pos, true);    // X, Y position

  Measure_Torque(1);
  Measure_Torque(2);
  
  if (2000+t < millis()) {
    t = millis();

    x_pos = 10;
    y_pos = 10;
    
    //x_pos = Trajectory[k][0];
    //y_pos = Trajectory[k][1];
    k += 1;

    if (k > 8) k=0;
    
  }
}

void task_4(float x_pos, float y_pos) {
  // Behave like spring using PI tuning

  Serial.print("Trajectory: "); 
  Serial.print(x_pos); 
  Serial.print(", "); 
  Serial.print(y_pos); 
  Serial.println(); 

  //FK(i, j);   // Angle based control
  IK_PI_control(x_pos, y_pos, true);    // X, Y position based control
}
