/* Analog Joystick

Connections: VRx to A0,  VRy to A1, +5V to 5, and GND to GND

  */
 #include <math.h>  
 
 int ledPin = 13;
 int joyPin1 = 0;                 // slider variable connecetd to analog pin 0
 int joyPin2 = 1;                 // slider variable connecetd to analog pin 1
 int x = 0;                  // variable to read the value from the analog pin 0
 int y = 0;                  // variable to read the value from the analog pin 1
 double offset1;
 double offset2; 
 int resolution = 20;
 double angle; 

void setup() {
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);              // initializes digital pins 0 to 7 as outputs
  delay(25);
  offset1=analogRead(joyPin1);
  //delay(25);
  offset2=analogRead(joyPin2);  
}

int treatValue(int data) {
  return (data * (2*resolution+1) / 1024);
 }

int treatAngle(int data){
  return (data/2);
}

 void loop() {
  // reads the value of the variable resistor 
  y = round(analogRead(joyPin1)-offset1);   
  // this small pause is needed between reading
  // analog pins, otherwise we get the same value twice
  delay(25);             
  
  // reads the value of the variable resistor 
  x = round(analogRead(joyPin2)-offset2); 

  delay(25);
  angle = round(atan2(x,y)*180/M_PI);
  
  Serial.print("x: "); 
  Serial.print(treatValue(x));
  Serial.print("\ty: ");
  Serial.println(treatValue(y));    
  Serial.print("\tAngle:");
  Serial.println(treatAngle(angle));
 }
