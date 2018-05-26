/* Sail Controller - Modified 5/26 M.S. 
 *  Autonomous sail control using anemometer and servo
 *  Connections: 
 *  Anemometer: A4 for Wind Direction, Pin 2 for Wind Speed
 *  Sail Servo: Signal to Pin 7
  */

// libraries 
#include <math.h>  

// Servo libraries 
#include <Servo.h> 

// assumed from rudder controller 
int currHeading = 0;          // current heading relative to boat (0 to 180 clockwise, 0 to -179 counterclockwise)
int prevHeading = 0;          // previous heading relative to boat 

// anemometer wind speed
#define WindSensorPin (2) // The pin location of the anemometer sensor 
int VaneValue;            // raw analog value from wind vane
int CalDirection;         // apparent wind direction: [0,180] clockwise, [0,-179] counterclockwise, 180 is dead zone

// anemometer wind direction 
float WindSpeed;                          // apparent wind speed in knots 
double T = 3.0;                           // time in seconds needed to average readings
unsigned long WindSpeedInterval = T*1000; // time in milliseconds needed to average anemometer readings 
unsigned long previousMillis = 0;         // millis() returns an unsigned long
volatile unsigned long RotationsCounter;  // cup rotation counter used in interrupt routine 
volatile unsigned long ContactBounceTime; // timer to avoid contact bounce in interrupt routine 

// Sail Servo Init
Servo servoS;                    // servo object for sail servo 
int turnAngle = 45;              // turn angle in degrees to determine straight sailing or turns
int sDesired = 0;                // desired sail angle relative to nose, uncalibrated  
int sZero = 60;                  // degrees required to zero sail servo in line with nose of boat
int sLimit = 90;                 // constraint angle limit to 90 degrees for the sail, determined by max slack allowed by rope length 
int sOffset;                     // calculated offset from getSailOffset necessary to map servo commands to sail angle (0 to 90 relative to boat nose) 
int sCommand;                    // calibrated angle command in degrees to servo.write 
int spSail;                      // set point sail angle relative to wind direction 

// Anemometer Wind Direction Prototyping
int getWindDirection(int VaneValue);           // anemometer requires calibration using curve fit data

// Sail Servo Prototyping
int getSailOffset(int sDesired);               // 0.85*sDesired + 60; used with getSailServoCommand
int getSailServoCommand(int sDesired);         // calibrate desired sail angle to angle command to servo.Write

void setup() {
  Serial.begin(9600); // baud rate (the number of times a signal in a communications channel changes state)
  
  // Anemometer Wind Direction Setup
  pinMode(WindSensorPin, INPUT); 
  
  // Anemometer Wind Speed Setup 
  attachInterrupt(digitalPinToInterrupt(WindSensorPin), isr_rotation, FALLING); 
  sei(); // Enables interrupts 

  // Sail Servo Setup
  servoS.attach(7);      // pin for the servoS control
  servoS.write(sOffset); // initialize at zero position 
  
}

void loop() {
  // Anemometer Wind Direction Loop
  VaneValue = analogRead(A4); 
  CalDirection = getWindDirection(VaneValue);

  // Curve fit outputs maximum of 174 degrees and minimum of -172, so set bounds as 180 and -179
  if (CalDirection >= 174)
  CalDirection = 180; 
  if(CalDirection <= -172) 
  CalDirection = -179; 
  
  // Anemometer Wind Speed Loop 
  unsigned long currentMillis = millis(); // current run time
  if ((unsigned long)(currentMillis - previousMillis) >= WindSpeedInterval){  // calculate wind speed after 3 seconds 
    // convert to knots using the formula V = P(2.25/T) / 1.15078 
    WindSpeed = RotationsCounter*2.25/T/1.15078;
    RotationsCounter = 0;                 // reset RotationsCounter after averaging  
    previousMillis = millis();
    }
  
  // Sail Servo Loop 
  if ((abs(prevHeading - currHeading)) < turnAngle){           // if the trajectory turns the boat less than the turnAngle degrees, then maintain 90 degree relative sail angle
    spSail = 90;                                  // set point sail angle relative to wind direction
    sDesired = abs(abs(CalDirection) - spSail);   // CalDirection [-179,180], sail doesn't care about direction   
  }
  else {
    sDesired = sLimit*abs(CalDirection)/180;      // from Davi sail Control Law, executed on turns 
  }
  sCommand = getSailServoCommand(sDesired);       // calibrate desired sail angle to angle command for servo
  servoS.write(sCommand);                         // command sail servo
  prevHeading = currHeading;                      // current heading becomes previous heading 
  }

// Servo Function definitions
int getSailOffset(int sDesired){            // used in getSailServoCommand function 
  return sDesired*0.85 + sZero;             // curve fit of data 
}

int getSailServoCommand(int sDesired){
  if(sDesired >= sLimit) {  // for desired sail angles greater than the limit, return the limit
    sOffset = getSailOffset(sDesired);
    return round(sLimit + sOffset);
    }
  else {                    // for all other sail angles, return the calibrated sail angle 
    sOffset = getSailOffset(sDesired);
    return round(sDesired + sOffset); 
    }
}

// Anemometer function definitions 
// function that the interrupt calls to increment the rotation count 
void isr_rotation () { 
if ((millis() - ContactBounceTime) > 15 ) { // debounce the switch contact.
  RotationsCounter++; 
  ContactBounceTime = millis(); 
  }
  }

// potentiometer values mapped to [-179,180], curve fit from data
int getWindDirection(int VaneValue){     
  return 0.3387*VaneValue - 172.4; ; 
}

