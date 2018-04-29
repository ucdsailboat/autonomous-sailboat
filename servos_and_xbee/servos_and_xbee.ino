/* Analog Joystick and Servo for the Rudder

Connections: VRx to A0,  VRy to A1, +5V to 5, and GND to GND
             Signal to Pin 6

  */


#include <Servo.h>
#include <SoftwareSerial.h>

//Sail servo init
Servo servoS;
int sDeg=0;
double sOffsetY;              
double sOffsetX;               
int sOffset= 40;      
int sLimit = 90;               //sail constrained to 0 to 90 deg rotation

//Rudder servo init
Servo servoR;                 
int rDeg=0;
double rOffsetY;               // initial value of X
double rOffsetX;               // initial value of Y
int rOffset = 130;            // offset to zero position (parallel with nose) for Laser rudder
int rLimit = 45;              // rudder constrained to -45 to 45 deg rotation 

void setup() {
  //XBee setup
  Serial2.begin(9600);
 
  // Sail servo setup 
  servoS.attach(6);
  servoS.write(sOffset);  //set initial servo position
  
  //Rudder sevo setup
  servoR.attach(7);
  servoR.write(rOffset); //set initial servo position
}

void loop() {
  servoS.write(Serial2.read())
  servoR.write(Serial2.read());
      
 }

