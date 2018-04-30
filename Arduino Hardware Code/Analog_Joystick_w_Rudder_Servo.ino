/* Analog Joystick and Servo for the Rudder

Connections: VRx to A0,  VRy to A1, +5V to 5, and GND to GND
             Signal to Pin 6

  */

// for joystick
#include <math.h>  
int ledPin = 13;
int joyPin1 = 0;              // slider variable connecetd to analog pin 0
int joyPin2 = 1;              // slider variable connecetd to analog pin 1
int x = 0;                    // variable to read the value from the analog pin 0
int y = 0;                    // variable to read the value from the analog pin 1
int resolution = 20;          // values scaled between -20 and 20 
double angle;                 // variable to store arctan of x and y
double offsetY;               // initial value of X
double offsetX;               // initial value of Y


// for servo
#include <Servo.h> 
Servo servoR;                 // creates servo object to control rudder servo 

int rOffset = 130;            // offset to zero position (parallel with nose) for Laser rudder
int deg = 0;                  // input angle into servo
int rLimit = 45;              // rudder constrained to -45 to 45 rotation 

void setup() {
  Serial.begin(9600);
  
  // for joystick
  pinMode(ledPin, OUTPUT);              // initializes digital pins 0 to 7 as outputs
  delay(75);
  offsetY = analogRead(joyPin1);
  offsetX = analogRead(joyPin2);  


  // for servo
  servoR.attach(6);  //the pin for the servoa control
  //servoS.attach(7);  //the pin for the servob control
  
  servoR.write(rOffset); //set initial servo position
  //servoS.write(10); //set initial servo position
}

// potentiometer value to between -20 and 20
int treatValue(int data) {
  return (data * (2*resolution+1) / 1024);
 }

// contrain rudder angle between -45 and 45
int treatAngle(int data) {
  if (data>=-90 && data<=90)
  {
    return (data/2);  
  }
  else if (data>90)
  {
    return abs(data/2 - 90);
    }
    else 
    {
      return abs(data/2) - 90;
    }
}

void loop() {
  // for joystick 
  // reads the value of the variable resistor 
  y = treatValue(round(analogRead(joyPin1)-offsetY));   
  // this small pause is needed between reading
  // analog pins, otherwise we get the same value twice
  delay(25);             
  
  // reads the value of the variable resistor 
  x = treatValue(round(analogRead(joyPin2)-offsetX)); 

  delay(25);
  angle = round(atan2(x,y)*180/M_PI);


  // for servo 
  deg = treatAngle(angle);  //convert readString into a number

  // rudder fixed at 45 for angles greater than 45 and -45 for angles less than -45
  if(deg >= rLimit) {
    Serial.print("writing Angle: ");
    Serial.println(45);
    servoR.write(rLimit+rOffset);
    }
  else if(deg <= -rLimit) {
      Serial.print("writing Angle: ");
      Serial.println(-45);
      servoR.write(-rLimit+rOffset);
    }
  else {   
    Serial.print("writing Angle: ");
    Serial.println(deg);
    servoR.write(deg + rOffset);
        }
      
  Serial.print("\tAngle:");
  Serial.println(treatAngle(angle));
 }
