// AUTHOR: Rob Tillaart
// PURPOSE: demo to measure mA DC
// URL: https://github.com/RobTillaart/ACS712


#include "ACS712.h"
#include "CytronMotorDriver.h"

//  Arduino UNO has 5.0 volt with a max ADC value of 1023 steps
//  ACS712 5A  uses 185 mV per A
//  ACS712 20A uses 100 mV per A
//  ACS712 30A uses  66 mV per A


// ACS712  ACS(A0, 5.0, 1023, 100);    // ---> Arduino
// ESP 32 example (might requires resistors to step down the logic voltage)

// ACS712 OUT - 34 (LINK1)
// ACS712 OUT - 35 (LINK2)

ACS712  ACS(35, 5, 4095, 100);   // ---> ESP
CytronMD motor2(PWM_DIR, 13, 12); 

void setup()
{
  Serial.begin(115200);
  ACS.autoMidPoint();
}

void loop()
{
  motor2.setSpeed(60);
  int mA = ACS.mA_DC();
  Serial.println(mA);
  delay(1000);
}
// -- END OF FILE --
