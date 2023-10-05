#include <ESP32Encoder.h>

#include "Config.h"
#include "Sensors.h"
#include "Motor.h"
#include "Torque_control.h"

#include "Kinematics.h"
#include "Tasks.h"


void setup() {
  Serial.begin(115200);
  
  pinMode(LED, OUTPUT);
  pinMode(M1_dir, OUTPUT);
  pinMode(M2_dir, OUTPUT);

  
  encoder1.attachHalfQuad(M1_DT_PIN, M1_SW_PIN);
  encoder2.attachHalfQuad(M2_DT_PIN, M2_SW_PIN);
  encoder1.setCount(0);
  encoder2.setCount(0);

  ledcSetup(ch1, freq, resolution);
  ledcSetup(ch2, freq, resolution);
  ledcAttachPin(M1_pwm, ch1);
  ledcAttachPin(M2_pwm, ch2);

  
  digitalWrite(M1_dir, LOW);
  digitalWrite(M2_dir, LOW);
  
  // Motor PWM
  ledcWrite(ch1, 0);   
  ledcWrite(ch2, 0); 

  delay(2000);
  Serial.println("Initialized...");
}



void loop() {
  
  // Task-1: Position control
  // Task-2: Trajectory control
  // Task-3: Const Force on wall using Torque control
  // Task-4: Behave like spring using PI tuning
  
  task_1(100, 100);
  //task_2();
  //task_3();
  //task_4();
  
  delay(10);
  Serial.println(); 
}
