#include <math.h>  
#include <SoftwareSerial.h>

int resolution = 20;          // joystick values scaled between -20 and 20 

// Sail joystick init
int sLedPin = 13;
int sJoyPin1 = 0;              // slider variable connecetd to analog pin 0
int sJoyPin2 = 1;              // slider variable connecetd to analog pin 1
int sX = 0;                    // variable to read the value from the analog pin 0
int sY = 0;                    // variable to read the value from the analog pin 1
double sAngle;                 // variable to store arctan of x and y

// Rudder joystick init
int rLedPin = 13;
int rJoyPin1 = 0;              // slider variable connecetd to analog pin 0
int rJoyPin2 = 1;              // slider variable connecetd to analog pin 1
int rX = 0;                    // variable to read the value from the analog pin 0
int rY = 0;                    // variable to read the value from the analog pin 1
double rAngle;                 // variable to store arctan of x and y

//XBee init
SoftwareSerial XBee(2,3); // RX, TX

void setup() {
  Serial.begin(9600);
  
  //Sail jouystick setup
  pinMode(sLedPin, OUTPUT);              // initializes digital pins 0 to 7 as outputs
  sOffsetY = analogRead(sJoyPin1);
  delay(5);
  sOffsetX = analogRead(sJoyPin2);  
  
  // Rudder joystick setup 
  pinMode(rLedPin, OUTPUT);              // initializes digital pins 0 to 7 as outputs
  rOffsetY = analogRead(rJoyPin1);
  delay(5);
  rOffsetX = analogRead(rJoyPin2);  

  }

// potentiometer value to between -(resolution) and +(resolution)
int treatValue(int data) {
  return (data * (2*resolution+1) / 1024);
 }

// constrain sail angle between 0 and 90
int sTreatAngle(int data){
  if (data <= 180 && data >= 0) {
  return data/4;
  }
  else {
    return (data/4 + 90);
  }
  }

  void loop() {
  // Read sail joystick and calculate heading
  sY = treatValue(round(analogRead(sJoyPin1)-sOffsetY));   
  delay(25);                       // this small pause is needed between reading analog pins, otherwise we get the same value twice
  sX = treatValue(round(analogRead(sJoyPin2)-sOffsetX));
  
  sAngle = round(atan2(sX,sY)*180/M_PI);
  sDeg = sTreatAngle(sAngle);  
  sOffset = 40 - deg/12;              //recalculate sail offset with correction 

   //Accounts for any sail degrees greater than 90
  if(sDeg >= sLimit) {
    Serial.println(sLimit+sOffset);
    }
  else {
    Serial.println(sDeg+sOffest);
    }
  }

  // Read rudder joystick and calculate heading
  rY = treatValue(round(analogRead(rJoyPin1)-rOffsetY));   
  delay(25);                       // this small pause is needed between reading analog pins, otherwise we get the same value twice
  rX = treatValue(round(analogRead(rJoyPin2)-rOffsetX));
  
  rAngle = round(atan2(rX,rY)*180/M_PI);
  rDeg = rTreatAngle(rAngle);  
  
  // rudder fixed at 45 for angles greater than 45 and -45 for angles less than -45
  if(rDeg >= rLimit) {
    Serial.println(rLimit);
    }
  else if(rDeg <= -rLimit) {
      Serial.println(-rLimit+rOffset);
    }
  else {   
    Serial.println(rDeg+rOffset);
        }
      
 }

