/*
Current sensor : 
*/


float getVPP(int sensorIn) {
  float result;
  int readValue;                // value read from the sensor
  int maxValue = 0;             // store max value here
  int minValue = 4096;          // store min value here ESP32 ADC resolution
  
   uint32_t start_time = millis();
   while((millis()-start_time) < 20) //sample for 0.2 Sec
   {
       readValue = analogRead(sensorIn);
       // see if you have a new maxValue
       if (readValue > maxValue) 
       {
           /*record the maximum sensor value*/
           maxValue = readValue;
       }
       if (readValue < minValue) 
       {
           /*record the minimum sensor value*/
           minValue = readValue;
       }
   }
   
   // Subtract min from max
   result = ((maxValue - minValue) * 3.3)/4096.0; //ESP32 ADC resolution 4096
      
   return result;
}

float Measure_Torque(int link) {

  float mot, ar;
  if (link == 1) {
    mot = M1_cs;
  } else if (link == 2) {
    mot = M2_cs;
  } else return 0;

  
  ar = analogRead(mot);
  
  Voltage = getVPP(mot);
  VRMS = (Voltage/2.0) *0.707;   //root 2 is 0.707
  AmpsRMS = ((VRMS * 1000)/mVperAmp)-0.75; //0.3 is the error I got for my sensor

  Torque = AmpsRMS;

  
  Serial.print("Link-");
  Serial.print(link);
  Serial.print(": Current = ");
  Serial.print(AmpsRMS);
  Serial.print(" ,Torque = ");
  Serial.println(Torque);

  return Torque;
}

void Torque_control_v1(int link, float q, float Torque_d) {
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
    
    Serial.println("Increse...");
  } else if (q<pos) {
    digitalWrite(dir, HIGH);
    
    Serial.println("Decrese...");
  } else {
    ledcWrite(mot, 0);   
  }

  //pwm -= 0.01;
  Serial.println(e);


  // If Torque is less tha desired one, apply force by pwm

  Torque = Measure_Torque(link);
  float T_err = Torque - Torque_d;
  if (T_err > 0) {
    ledcWrite(mot, 0); 
  } else {
    ledcWrite(mot, 0); 
  }
}
