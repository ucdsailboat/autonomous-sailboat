/* Sail Controller 
 *  Autonomous sail control using anemometer and servo
 *  Connections: 
 *  Joystick: VRx to A0,  VRy to A1, +5V to 5, and GND to GND
 *  Anemometer: A4 for Wind Direction, Pin 2 for Wind Speed
 *  Sail Servo: Signal to Pin 7
  */

// libraries 
#include <math.h>  
#include <Servo.h> 

// assumed from rudder controller
int h1Delta = 0;          // current heading relative to boat
int h2Delta = 0;          // previous heading relative to boat 

// Anemometer Wind Direction Init  
#define WindSensorPin (2) // The pin location of the anemometer sensor 
int VaneValue;            // raw analog value from wind vane
int CalDirection;         // apparent wind direction (0 to 360 degrees where 0/360 is dead zone)

// Anemometer Wind Speed Init 
int Rotations;                            // displayed onto Serial Monitor 
float WindSpeed;                          // apparent wind speed in knots 
double T = 3.0;                           // time in seconds needed to average readings
unsigned long WindSpeedInterval = T*1000; // time in milliseconds needed to average anemometer readings 
unsigned long previousMillis = 0;         // millis() returns an unsigned long
volatile unsigned long RotationsCounter;  // cup rotation counter used in interrupt routine 
volatile unsigned long ContactBounceTime; // timer to avoid contact bounce in interrupt routine 

// Joystick Init (remove for final)
int ledPin = 13;
int joyPin1 = 0;            // slider variable connecetd to analog pin 0
int joyPin2 = 1;            // slider variable connecetd to analog pin 1
int x = 0;                  // raw values from analog pin 0
int y = 0;                  // raw values from analog pin 1
double offsetX;             // initial nonzero reading
double offsetY;             
int resolution = 20;        // map joystick outputs from -20 to +20
double angle;               // arctan2(x,y) in degrees (0 to 180 clockwise, 0 to -179 couterclockwise)

// Sail Servo Init
Servo servoS;               // servo object for sail servo 
int deg = 0;                // desired heading 
int sOffset = 40 - deg/12;  // offset necessary to map servo commands to sail angle (0 to 20 relative to boat nose)  
int sLimit = 90;            // constraint angle limit to 90 degrees for the sail 
int sCommand;               // nominal angle command in degrees to servo.write 
int spSail                  // set point sail angle relative to wind direction 

// Anemometer Wind Direction Prototyping
void getHeading(int direction);       // defines ranges for compass rose

// Joystick Prototying (remove for final)
int treatValue(int data);             // map raw joystick values to -20 to +20 in x and y directions

// Sail Servo Prototyping
int treatAngle(int data);             // convert data in [-179,180] to [0,90] 
int commandSailServo(int deg);

void setup() {
  Serial.begin(9600); // baud rate (the number of times a signal in a communications channel changes state)

  // Anemometer Wind Direction Setup
  pinMode(WindSensorPin, INPUT); 
  attachInterrupt(digitalPinToInterrupt(WindSensorPin), isr_rotation, FALLING); 
  sei(); // Enables interrupts 
  Serial.println("Vane Value\tDirection\tHeading\t\tRotations\tKnots\tSail Servo Angle"); 

  // Joystick Setup (remove for final)
  pinMode(ledPin, OUTPUT);              // initializes digital pins 0 to 7 as outputs
  delay(75);
  offsetY=analogRead(joyPin1);
  offsetX=analogRead(joyPin2);  

  // Sail Servo Setup
  servoS.attach(7);  // pin for the servoS control
  servoS.write(sOffset); // initialize at zero position 
  
}


void loop() {
  // Anemometer Wind Direction Loop
  VaneValue = analogRead(A4); 
  CalDirection = map(VaneValue, 0, 1023, 0, 360);

  if(CalDirection > 360) 
  CalDirection = CalDirection - 360; 

  if(CalDirection < 0) 
  CalDirection = CalDirection + 360; 
 
  //Wind speed
  unsigned long currentMillis = millis(); // current run time
  if ((unsigned long)(currentMillis - previousMillis) >= WindSpeedInterval){  // calculate wind speed after 3 seconds 
    // convert to knots using the formula V = P(2.25/T) / 1.15078 
    WindSpeed = RotationsCounter*2.25/T/1.15078;
    Rotations = RotationsCounter;                       
    RotationsCounter = 0;         // reset RotationsCounter after averaging  
    previousMillis = millis();
    }
    
    Serial.print(VaneValue); Serial.print("\t\t");
    Serial.print(CalDirection); Serial.print("\t\t"); 
    getHeading(CalDirection); Serial.print("\t\t"); 
    Serial.print(Rotations); Serial.print("\t\t"); 
    Serial.println(WindSpeed); Serial.print("\t\t");

  
  // Joystick Loop (remove for final) 
  y = treatValue(round(analogRead(joyPin1)-offsetY));  
  delay(25);    
  x = treatValue(round(analogRead(joyPin2)-offsetX)); 
  delay(25);
  angle = round(atan2(x,y)*180/M_PI);   // convert x and y coordinates into angle in degrees
  h1Delta = angle;          // use joystick as a heading input, comment out when using anemometer
  
  // Sail Servo Loop 
  deg = treatAngle(h1Delta);  // convert heading input to sail angle between 0 and 90 degrees
  if ((h2Delta - h1Delta) < 45){
    spDelta = 90;             // set point sail angle relative to wind direction
    sDelta = abs(CalDirection) - spDelta;    // revise: CalDirection [0,360], not [-179,180] or change anemometer code 
  }
  else {
    sDelta = 90*abs(CalDirection)/180;      // revise: CalDirection [0,360], not [-179,180]
  }
  commandSailServo(sDelta);
  h2Delta = h1Delta;
  }

// Joystick Function defintions (remove for final)
// Changes potentiometer values into +/- resolution 
int treatValue(int data) {
  return (data * (2*resolution+1) / 1024);
 }
 
// Servo Function definitions
// constrain sail angle between 0 and 90 degrees 
// data: heading angle (0 to 180 clockwise and 0 to -179 counterclockwise)
// return sail angle command [0,90]  
int treatAngle(int data){
  if (data <= 180 && data >= 0) {
  return data/4;
  }
  else {
    return (data/4 + 90);
  }
  }

int commandSailServo(int deg){
  if(deg >= sLimit) {
    // note: 40 (deg) is the offset to zero
    sOffset = 40 - deg/12;
    Serial.println(sLimit); Serial.print("\t\t");
    servoS.write(sLimit+sOffset);
    }
   // runs if degrees are within the sLimit
  else {
    // note: 40 (deg) is the offset
    sOffset = 40 - deg/12;
    Serial.print("\t\t");
    Serial.println(deg);
    servoS.write(deg+sOffset);
    }
}

// Anemometer function definitions 
// This is the function that the interrupt calls to increment the rotation count 
void isr_rotation () { 
if ((millis() - ContactBounceTime) > 15 ) { // debounce the switch contact. 
RotationsCounter++; 
ContactBounceTime = millis(); 
}
}

void getHeading(int direction) { 
if(direction < 22) 
Serial.print("N"); 
else if (direction < 67) 
Serial.print("NE"); 
else if (direction < 112) 
Serial.print("E"); 
else if (direction < 157) 
Serial.print("SE"); 
else if (direction < 212) 
Serial.print("S"); 
else if (direction < 247) 
Serial.print("SW"); 
else if (direction < 292) 
Serial.print("W"); 
else if (direction < 337) 
Serial.print("NW"); 
else 
Serial.print("N"); 
}
