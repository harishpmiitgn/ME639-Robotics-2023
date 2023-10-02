#include <Arduino.h>

// 26 AND 25 enocder link1  
// 33 and 32 encoder link2
const int encoderPin1A = 32;   
const int encoderPin1B = 33;   
const int encoderPin2A = 25; 
const int encoderPin2B = 26; 
const int pulsesPerRevolution = 8000; // Assuming 200 PPR for the AMT1120

volatile int encoderPosition1 = 0;
volatile int encoderPosition2 = 0;

void setup() {
  Serial.begin(9600);

  pinMode(encoderPin1A, INPUT_PULLUP);
  pinMode(encoderPin1B, INPUT_PULLUP);

  pinMode(encoderPin2A, INPUT_PULLUP);
  pinMode(encoderPin2B, INPUT_PULLUP);

  // Attach interrupt handlers for encoder pulses
  attachInterrupt(digitalPinToInterrupt(encoderPin1A), handleEncoder1Interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin2A), handleEncoder2Interrupt, CHANGE);
  
}

void loop() {
  // Convert encoder position to degrees
  float degrees1 = (static_cast<float>(encoderPosition1) / pulsesPerRevolution) * 360.0;
  float degrees2 = (static_cast<float>(encoderPosition2) / pulsesPerRevolution) * 360.0;
  // Print the current encoder position in degrees
  Serial.print("1 Encoder Position (Degrees): ");
  Serial.println(degrees1);
  Serial.print("2 Encoder Position (Degrees): ");
  Serial.println(degrees2);
  delay(2000); // Delay for readability (adjust as needed)
}

void handleEncoder1Interrupt() {
  // Read the current state of encoderPin1A
  int state1A = digitalRead(encoderPin1A);
  // Read the current state of encoderPin1B
  int state1B = digitalRead(encoderPin1B);
  // Determine the direction of rotation based on the states of A and B
  int direction1 = (state1A == state1B) ? 1 : -1;
  // Update the encoder position
  encoderPosition1 += direction1;

}

void handleEncoder2Interrupt() {
  // Read the current state of encoderPin1A
  int state2A = digitalRead(encoderPin2A);
  // Read the current state of encoderPin1B
  int state2B = digitalRead(encoderPin2B);
  // Determine the direction of rotation based on the states of A and B
  int direction2 = (state2A == state2B) ? 1 : -1;
  // Update the encoder position
  encoderPosition2 += direction2;
}

