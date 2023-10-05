
#define   LED         2     // Onboard LED pin

// Encoder Pins
#define   M1_DT_PIN   32    // Motor-1 rotary encoder's DT pin
#define   M1_SW_PIN   33    // Motor-1 rotary encoder's SW pin
#define   M2_DT_PIN   25    // Motor-2 rotary encoder's DT pin
#define   M2_SW_PIN   26    // Motor-2 rotary encoder's SW pin


// Motor Driver Pins
#define   M1_dir      12    // Motor-1 Direction pin (connected through optocoupler)
#define   M1_pwm      13    // Motor-1 PWM pin (connected through optocoupler)
#define   M2_dir      27    // Motor-2 Direction pin (connected through optocoupler)
#define   M2_pwm      14    // Motor-2 PWM pin (connected through optocoupler)

// Motor & Encoder Parameters
#define   KVA         3.7
#define   ENC_PULSE   8000

// Current Sensor Pins
#define   M1_cs       35    // Motor-1 Current sensor 
#define   M2_cs       34    // Motor-2 Current sensor 


// Manipulator static & dynamic Parameter
const float l1 = 155, l2 = 155;   // mm
double t = 0, i = 0, j = 0 ;

// PWM channel Parameters
const int freq = 50;
const int ch1 = 0;
const int ch2 = 1;
const int resolution = 8;

// Current & Torque sensor Parameters - apply Kalman filter
int mVperAmp = 100;   // {5A - 185,   20A - 100,  30A - 66} for version of ACS712
int Watt = 0;
double Voltage = 0;
double VRMS = 0;
double AmpsRMS = 0;
double Torque = 0;

int M1_ar, M2_ar;

volatile int M1_enc = 0; // Global variable for storing the encoder position
