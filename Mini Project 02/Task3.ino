#include <Arduino.h>
#include "CytronMotorDriver.h"
#include <Encoder.h>

CytronMD motor1(PWM_DIR, 14, 27);
CytronMD motor2(PWM_DIR, 13, 12);

const int encoder1PinA = 26;
const int encoder1PinB = 25;

const int encoder2PinA = 33;
const int encoder2PinB = 32;

const int pulsesPerRevolution = 8000;

volatile int encoder1Position = 0;
volatile int encoder2Position = 0;

float motor_speed_1 = 0;
float motor_speed_2 = 0;

const float Kp1 = 0.51;     // Proportional gain
const float Ki1 = 0.00;     // Integral gain
const float Kd1 = 0.03;      // Derivative gain

const float Kp2 = 0.60;     // Proportional gain
const float Ki2 = 0.00;     // Integral gain
const float Kd2 = 0.03;      // Derivative gain

float previousError1 = 0;
float integral1 = 0;

float previousError2 = 0;
float integral2 = 0;

float perturbationFactor = 70;  // Perturbation factor
float amplitude1 = 100;  // Amplitude of oscillation for motor 1
float amplitude2 = 100;  // Amplitude of oscillation for motor 2
float decayRate = 0.95;  // Decay rate for reducing amplitude

void setup() {
  Serial.begin(9600);

  pinMode(encoder1PinA, INPUT_PULLUP);
  pinMode(encoder1PinB, INPUT_PULLUP);
  pinMode(encoder2PinA, INPUT_PULLUP);
  pinMode(encoder2PinB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoder1PinA), handleEncoder1Interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder2PinA), handleEncoder2Interrupt, CHANGE);

  // motor1.begin();
  // motor2.begin();
}

void loop() {
  float currentAngle1 = (static_cast<float>(encoder1Position) / pulsesPerRevolution) * 360.0;
  float currentAngle2 = (static_cast<float>(encoder2Position) / pulsesPerRevolution) * 360.0;

  // Calculate perturbation
  float perturbation1 = perturbationFactor * sin(millis() * 0.001);
  float perturbation2 = perturbationFactor * cos(millis() * 0.001);

  // Update target angles with perturbation
  float targetAngle1 = amplitude1 * sin(millis() * 0.001) + perturbation1;
  float targetAngle2 = amplitude2 * cos(millis() * 0.001) + perturbation2;

  // Calculate errors
  float error1 = targetAngle1 - currentAngle1;
  float error2 = targetAngle2 - currentAngle2;

  // Update amplitudes to decay over time
  amplitude1 *= decayRate;
  amplitude2 *= decayRate;

  // Calculate PID terms for motor 1
  float proportional1 = Kp1 * error1;
  integral1 += Ki1 * error1;
  float derivative1 = Kd1 * (error1 - previousError1);

  motor_speed_1 = proportional1 + integral1 + derivative1;

  // Apply constraints to motor speed
  if (motor_speed_1 > 80) {
    motor_speed_1 = 70;
  }

  // Update previous error for motor 1
  previousError1 = error1;

  // Calculate PID terms for motor 2
  float proportional2 = Kp2 * error2;
  integral2 += Ki2 * error2;
  float derivative2 = Kd2 * (error2 - previousError2);

  motor_speed_2 = proportional2 + integral2 + derivative2;

  // Apply constraints to motor speed
  if (motor_speed_2 > 80) {
    motor_speed_2 = 70;
  }

  // Update previous error for motor 2
  previousError2 = error2;

  // Update motor speeds
  motor1.setSpeed(motor_speed_1);
  motor2.setSpeed(motor_speed_2);

  // Print information
  Serial.print("Encoder Position (Degrees1): ");
  Serial.println(currentAngle1);
  Serial.print("Error1: ");
  Serial.println(error1);
  Serial.print("Encoder Position (Degrees2): ");
  Serial.println(currentAngle2);
  Serial.print("Error2: ");
  Serial.println(error2);
  Serial.print("Amplitude1: ");
  Serial.println(amplitude1);
  Serial.print("Amplitude2: ");
  Serial.println(amplitude2);
  Serial.println(encoder1Position);
  delay(10);
}

void handleEncoder1Interrupt() {
  int stateA = digitalRead(encoder1PinA);
  int stateB = digitalRead(encoder1PinB);
  int direction = (stateA == stateB) ? 1 : -1;
  encoder1Position += direction;
}

void handleEncoder2Interrupt() {
  int stateA = digitalRead(encoder2PinA);
  int stateB = digitalRead(encoder2PinB);
  int direction = (stateA == stateB) ? 1 : -1;
  encoder2Position += direction;
}