

ESP32Encoder encoder1;     //8000 pulses per rotation
ESP32Encoder encoder2;     //8000 pulses per rotation

float M1_encoder() {
  long M1_Pos = encoder1.getCount();
  return -360*M1_Pos/8000;
}

float M2_encoder() {
  long M2_Pos = encoder2.getCount();
  return -360*M2_Pos/8000;
}

float M1_current() {
  
}

float M2_current() {
  
}

float Get_torque(float curr) {
  return curr/KVA;
}
