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
int y = 0;                   // variable to read the value from the analog pin 1
double angle;
double offsety;
double offsetx;  
int resolution = 20;

// for servo
String readString;
#include <Servo.h> 
Servo myservoa; // creates servo object to control sail servo 

int roffset = 130;


void setup() {
  Serial.begin(9600);
  
  // for joystick
  pinMode(ledPin, OUTPUT);              // initializes digital pins 0 to 7 as outputs
  delay(75);
  offsety = analogRead(joyPin1);
  offsetx = analogRead(joyPin2);  


  // for servo
  myservoa.write(roffset); //set initial servo position
  //myservob.write(10); //set initial servo position
  
  myservoa.attach(6);  //the pin for the servoa control
  //myservob.attach(7);  //the pin for the servob control
}

int treatValue(int data) {
  return (data * (2*resolution+1) / 1024);
 }

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
  y = treatValue(round(analogRead(joyPin1)-offsety));   
  // this small pause is needed between reading
  // analog pins, otherwise we get the same value twice
  delay(25);             
  
  // reads the value of the variable resistor 
  x = treatValue(round(analogRead(joyPin2)-offsetx)); 

  delay(25);
  angle = round(atan2(x,y)*180/M_PI);


  // for servo 
  int n = treatAngle(angle);  //convert readString into a number

  // auto select appropriate value, copied from someone elses code.
  if(n>=45)
  {
    Serial.print("writing Angle: ");
    Serial.println(45);
    myservoa.write(45+roffset);
    }
    else if(n<=-45)
    {
      Serial.print("writing Angle: ");
      Serial.println(-45);
      myservoa.write(-45+roffset);
      }
      else 
      {   
        Serial.print("writing Angle: ");
        Serial.println(n);
        myservoa.write(n+roffset);
        }
      
  Serial.print("\tAngle:");
  Serial.println(treatAngle(angle));
 }
