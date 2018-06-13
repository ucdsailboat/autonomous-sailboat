/*WIND OPTIMIZATION AUTONOMOUS MODE: This runs after the boat is set within the predefined radius (maxRadius) of the predefined origin (origin[2]). 
  While this is running, the sensor data will update every interval of "display_timer", with the anemometer wind speed updating every "T" seconds.
  It sends servo commands to the rudder and sail 
*/

//LIBRARIES
#include <math.h>  

// GPS libraries 
#include <TinyGPS++.h>
//#include <SoftwareSerial.h>

//Rudder libraries
#include <stdio.h>
#include <stdlib.h>

//Servo library
#include <Servo.h>




//VARIABLE INITIALIZATIONS
//Data Logging Initialization
float display_timer = 500;                //time (ms) between printing to the serial port

// Anemometer Initialization
#define WindSensorPin (2) 
int VaneValue;                            // raw analog value from wind vane
int CalDirection;                         // apparent wind direction: [0,180] cw, [0,-179] ccw, 180 is dead zone
float WindSpeed;                          // apparent wind speed (knots)
double T = 3.0;                           // time in seconds needed to average readings
unsigned long WindSpeedInterval = T*1000; // time in milliseconds needed to average anemometer readings 
unsigned long previousMillis = 0;         // millis() returns an unsigned long
volatile unsigned long RotationsCounter;  // cup rotation counter used in interrupt routine 
volatile unsigned long ContactBounceTime; // timer to avoid contact bounce in interrupt routine 
uint32_t timer = millis();

//GPS Initialization
TinyGPSPlus tinyGPS;        // Create a TinyGPS object
#define GPS_BAUD 9600       // GPS module baud rate
#define gpsPort Serial1

//Wind Optimization Initialization 
float origin[2] = {38.537587, -121.748064};   //GPS coordinates of origin (latitude, longitude) sequoia apt
//float curLocation[2]={origin[0],origin[1]};   //GPS coordinates of current location
//float earthR= 6378100;      //m
//float a;                    //used for calculating haversine angle
float maxRadius = 7;          //m
float radius;               //current distance from origin
int againstWindAngle=180-45;  
float windAngle = 0;        //angle of wind in body frame (-180,180]
float desiredAngle =againstWindAngle;
boolean turning=0;
float previousMillisTurning;
float turningBuffer = 3000; // buffer to allow boat to turn and get back into boundary (ms)

//Rudder Initialization
int rOffset=103;
float rudderPos = 45+rOffset;      // positive rudder limit
float rudderNeg = -45+rOffset;     // negative rudder limit
float errorActual = 0;      // error = desired angle - actual angle 
float Iaccum = 0;           // accummulator of integrative error.
float Kp = 2;               // proportional gain - starting value suggested by Davi
float Ki = .001;               // integral gain - starting value suggsted by Davi
float controlAct = 0;      // initialize control action (P + I)
float heading=0;            // actual angle/direction
float angle_rudder = 0;     // angle of the rudder commanded 
float sample_time = 0.2;    // sample time of the system (in seconds)
int deg = 0;                // input angle into servo

Servo servoR;

//Sail Initialization 
int turnAngle = 45;               // turn angle in degrees to determine straight sailing or turns
int sDesired = 45;                 // desired sail angle to maintain relative to nose, uncalibrated  
int sZero = 60;                   // degrees required to zero sail servo in line with nose of boat
int sLimit = 90;                  // constraint angle limit to 90 degrees for the sail, determined by max slack allowed by rope length 
int sOffset=sZero - sDesired/12;  // calculated offset from getSailOffset necessary to map servo commands to sail angle (0 to 90 relative to boat nose) 
int sCommand;                     // calibrated angle command in degrees to servo.write 
int spSail;                       // set point sail angle relative to wind direction 
int currHeading = 0;                  // current heading relative to boat (0 to 180 clockwise, 0 to -179 counterclockwise)
int prevHeading = 0;                  // previous heading relative to boat 
Servo servoS;  

 

//FUNCTION PROTOTYPES
// Anemometer Wind Direction Prototypes
int getWindDirection(int VaneValue);      

//GPS Prototypes
void printInfo(void);

//WO Prototypes
float haversin(float angle);

//Rudder Prototypes
float saturator(float value);
float saturator_rudder(float servoRudderAngle);
float rudder_controller(float desiredPath, float heading);
float P();
float I();

//Sail Prototypes
int getSailOffset(int sDesired);               // 0.85*sDesired + 60; used with getSailServoCommand
int getSailServoCommand(int sDesired);         // calibrate desired sail angle to angle command to servo.Write




//SETUP 
void setup() {
  Serial.begin(9600); // min baud rate for GPS is 115200 so may have to adjust 
  
  // Anemometer Wind Direction Setup
  pinMode(WindSensorPin, INPUT); 
  
  // Anemometer Wind Speed Setup 
  attachInterrupt(0, isr_rotation, FALLING); 
  sei(); // Enables interrupts 
  
  //GPS Setup
  gpsPort.begin(GPS_BAUD); // GPS_BAUD currently set to 9600  

  //Rudder Setup
  servoR.attach(6);      // pin for the servoS control
  servoR.write(rOffset); // initialize at zero position 

  //Sail Setup
  servoS.attach(7);      // pin for the servoS control
  servoS.write(sZero); // initialize at zero position 
}





//LOOP 
void loop() {
  
  //[Update and Print Sensor Info]
  if (timer > millis())  timer = millis();                              //Reset timer if millis() or timer wraps around
  
  // Update wind speed
  if ((unsigned long)(millis()- previousMillis) >= WindSpeedInterval){  //Calculate wind speed after T seconds 
    // convert to knots using the formula V = P(2.25/T) / 1.15078 
    WindSpeed = RotationsCounter*2.25/T/1.15078;
    RotationsCounter = 0;                                               //Reset RotationsCounter after averaging  
    previousMillis = millis();
    }
    
  // Update wind direction
  VaneValue = analogRead(A4); 
  CalDirection = getWindDirection(VaneValue);
   
  if (CalDirection >= 174)                                              //Curve fit has values less than -179 
  CalDirection = 180; 
  if(CalDirection <= -172) 
  CalDirection = -179;                          
  
  //Print data to serial port
  if (millis() - timer > display_timer) {                               
    timer = millis();                                                   //Reset the timer    
    printInfo();                                                        //Print sensor data to serial port
  }
  smartDelay(50); // "Smart delay" looks for GPS data while the Arduino's not doing anything else




//[Wind Optimization]
  //curLocation[0]=tinyGPS.location.lat();   
  //curLocation[1]=tinyGPS.location.lng();   
  windAngle = CalDirection;
  
  //a = haversin((origin[0] - curLocation[0])*M_PI/180) + cos(origin[0]*M_PI/180)*cos(curLocation[0]*M_PI/180)*haversin((origin[1]- curLocation[1])*M_PI/180);
  //radius=2*earthR*atan2(sqrt(a),sqrt(1-a));                                           
  
  //Calculate radius from origin
  radius=tinyGPS.distanceBetween(origin[0],origin[1],tinyGPS.location.lat(),tinyGPS.location.lng());
  
  //If the boat is beyond maxRadius from the origin, determine which way to turn
  if (radius > maxRadius && !turning){  
    //Determine which way to turn                           
    if ( desiredAngle == againstWindAngle)      desiredAngle = 0;                  //CW turn to go with the wind
    else if(desiredAngle == 0)                desiredAngle = -againstWindAngle;    //CW turn to go -45 against the wind
    else if (desiredAngle == -againstWindAngle) desiredAngle = -1;                 //CCW turn to go with the wind
    else if(desiredAngle == -1)                desiredAngle = againstWindAngle;     //CW turn to go 45 against the wind 

    //Prep to turn                           
    previousMillisTurning=millis();
    turning=1;
    }
  
  //If the boat is turning, determine if the boat is still turning and/or out of bounds
  if (turning && radius<maxRadius && (millis()- previousMillisTurning)>turningBuffer){
      turning=0;          
    }




//[Rudder Controller]
  servoR.write(rudder_controller(desiredAngle, windAngle));   //Send this command to the rudder servo




//[Sail Controller]  
  if ((abs(prevHeading - currHeading)) < turnAngle){           // if the trajectory turns the boat less than the turnAngle degrees, then maintain 90 degree relative sail angle
    spSail = 90;                                  // set point sail angle relative to wind direction
    sDesired = abs(abs(CalDirection) - spSail);   // CalDirection [-179,180], sail doesn't care about direction   
  }
  else {
    sDesired = sLimit*abs(CalDirection)/180;      // from Davi sail Control Law, executed on turns 
  }
  sCommand = getSailServoCommand(sDesired);       // calibrate desired sail angle to angle command for servo
  servoS.write(sCommand);                         // command sail servo
  prevHeading = currHeading;                              // current heading becomes previous heading 
  if (sCommand > 140)
  sCommand = 140;                                 // maximum for servo
  servoS.write(sCommand);                         // command sail servo
  prevHeading = currHeading;                      // current heading becomes previous heading 

  //Serial.print(sCommand);
  //Serial.println(CalDirection);
  }




//FUNCTION DEFINITION
//Sensor Data Logging Function Definition
void printInfo(){
  //Print latitude (degs), longitude (degs), GPS speed (mph), wind direction (degs), wind speed (knots)
  Serial.print(tinyGPS.location.lat(), 8); Serial.print(","); 
  Serial.print(tinyGPS.location.lng(), 8); Serial.print(","); 
  Serial.print(tinyGPS.speed.mph()); Serial.print(",");
  Serial.print(CalDirection); Serial.print(","); 
  Serial.print(desiredAngle);Serial.print(",");
  Serial.println(WindSpeed); 

  //for testing
  //Serial.print("\t\t");Serial.print(errorActual);Serial.print("\t");Serial.print(desiredAngle);Serial.print("\t");Serial.print(turning);Serial.print("\t");Serial.println(radius);
}



//WO Function Definition
float haversin(float angle){                //radians
    return pow(sin(angle/2),2);
}


   
// Anemometer Function Definition
// Interrupt function to increment the rotation count 
void isr_rotation () { 
  if ((millis() - ContactBounceTime) > 15 ) {     // Debounce the switch contact.
    RotationsCounter++; 
    ContactBounceTime = millis(); 
  }
}
int getWindDirection(int VaneValue){              // Potentiometer values mapped to [-179,180], curve fit from data
  return 0.3387*VaneValue - 172.4; ;  
}



// GPS Function Definition
static void smartDelay(unsigned long ms){
  // This custom version of delay() ensures that the tinyGPS object is being "fed". From the TinyGPS++ examples.
  unsigned long start = millis();
  do{
    // If data has come in from the GPS module
    while (gpsPort.available())
      tinyGPS.encode(gpsPort.read());           // Translate from NMEA format
  } while (millis() - start < ms);
}



//Rudder Controller Definitions
float saturator(float value) {
  //Maps [0-360] to [-180,180]
  if (value > 180)  value = value - 360;
  if (value < -180) value = value + 360;
  return value;
}
float saturator_rudder(float servoRudderAngle) {
  //Keeps rudder angle within limits
  if (servoRudderAngle > rudderPos) servoRudderAngle = rudderPos;
  if (servoRudderAngle < rudderNeg) servoRudderAngle = rudderNeg;
  return servoRudderAngle;
}
float rudder_controller(float desiredPath, float heading) {
  desiredPath = saturator(desiredPath);           //same as desired angle
  errorActual = desiredPath - heading;            //error between the actual heading and desired path
  errorActual = saturator(errorActual);
  controlAct = P() + I();                        //PI control
  angle_rudder = saturator_rudder(rOffset + (rudderNeg-rOffset)*controlAct/180);
  return angle_rudder;
}
float P(){
  return Kp * errorActual;
}
float I() {                                         //error is reduced at each iteration                                 
  Iaccum = Iaccum + Ki * errorActual * sample_time;
  return Iaccum;
}



//Sail Controller Definitions
int getSailOffset(int sDesired){            // used in getSailServoCommand function 
  return sDesired*0.85 + sZero;             // curve fit of data 
}
int getSailServoCommand(int sDesired){
  if(sDesired >= sLimit) {  // for desired sail angles greater than the limit, return the limit
    sCommand = getSailOffset(sLimit);
    return round(sCommand);
    }
  else {                    // for all other sail angles, return the calibrated sail angle 
    sCommand = getSailOffset(sDesired);
    return round(sCommand); 
    }
}
