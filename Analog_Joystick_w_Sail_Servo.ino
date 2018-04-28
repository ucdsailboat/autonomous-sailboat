/*Sail Servo Code & with joystick inputs
//Description:  This script controls how the sail servo is controlled manually 
Joystick Connections: VRx to A0, VRy to A1, +5V to 5, & GND to GND */

#include <Servo.h> 
#include <math.h>  

String readString; // display input from serial 
Servo servoR; // creates servo object to control rudder servo
Servo servoS; // creates servo object to control sail servo 
int deg = 0; // initialize input degree
int sOffset = 40 - deg/12; //degrees 
int sLimit = 90; // constraint angle limit to 90 for the sail 

int ledPin = 13;
int joyPin1 = 0;                 // slider variable connecetd to analog pin 0
int joyPin2 = 1;                 // slider variable connecetd to analog pin 1
int x = 0;                  // variable to read the value from the analog pin 0
int y = 0;                  // variable to read the value from the analog pin 1
double offsetX;
double offsetY; 
int resolution = 20;
double angle; 

void setup() {
  Serial.begin(9600); // baud rate (the number of times a signal in a communications channel changes state)
  
  pinMode(ledPin, OUTPUT);              // initializes digital pins 0 to 7 as outputs
  delay(75);
  offsetY=analogRead(joyPin1);
  offsetX=analogRead(joyPin2);  
  
  servoR.attach(6);  //the pin for the servoa control
  servoS.attach(7);  //the pin for the servob control
  servoS.write(sOffset);
  
}

// Changes potentiometer values into +/- resolution 
int treatValue(int data) {
  return (data * (2*resolution+1) / 1024);
 }

// contrain sail angle between 0 and 90 degrees 
// input: data - inputted angle from the joystick (calculated with the x,y coordinate) 
int treatAngle(int data){
  if (data <= 180 && data >= 0) {
  return data/4;
  }
  else {
    return (data/4 + 90);
  }
  }


void loop() {
  y = treatValue(round(analogRead(joyPin1)-offsetY));  
  delay(25);    
  x = treatValue(round(analogRead(joyPin2)-offsetX)); 
  delay(25);
  angle = round(atan2(x,y)*180/M_PI);
  deg = treatAngle(angle);  //convert readString into a number
  
  //accounts for any degrees greater than 90
  if(deg >= sLimit) {
    Serial.print("writing Angle: ");
    // note: 40 (deg) is the offset 
    sOffset = 40 - deg/12;
    Serial.println(sLimit);
    servoS.write(sLimit+sOffset);
    }
   // runs if degrees are within the sLimit
  else {
    Serial.print("writing Angle: ");
    // note: 40 (deg) is the offset
    sOffset = 40 - deg/12;
    Serial.println(deg);
    servoS.write(deg+sOffset);
    }
  }



//Notes

/*  // runs if there is an input into the Serial 
  if (Serial.available())  {
    char c = Serial.read();  //gets one byte from serial buffer
    // only runs if a comma is entered into the serial 
    if (c == ',') {
      if (readString.length() >1) {
        Serial.println(readString); //prints string to serial port out
        deg = readString.toInt();  //converts readString (type String) into deg (type Int)

        */

        
