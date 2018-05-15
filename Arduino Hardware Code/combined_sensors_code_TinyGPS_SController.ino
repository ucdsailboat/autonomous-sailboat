/* Anemometer and GPS Combined Code - Modified 5/13 M. Shi  
 * Anemometer loop is currently in the GPS timer loops that displays every 500ms (set in display_timer variable)
 * Anemometer readings averaged every 3000ms (set in WindSpeedInterval variable)
*/ 

// Servo libraries
#include <Servo.h> 

// assumed from rudder controller, from joystick for testing 
int h1Delta = 0;          // current heading relative to boat (0 to 180 clockwise, 0 to -179 counterclockwise)
int h2Delta = 0;          // previous heading relative to boat 

// Sail Servo Init
Servo servoS;                    // servo object for sail servo 
int turnAngle = 45;              // turn angle in degrees to determine straight sailing or turns
int sDesired = 0;                // desired sail angle relative to nose, uncalibrated  
int sZero = 40;                  // degrees required to zero sail servo in line with nose of boat
int sOffset = sZero - sDesired/12;  // offset necessary to map servo commands to sail angle (0 to 90 relative to boat nose)  
int sLimit = 90;                 // constraint angle limit to 90 degrees for the sail, determined by max slack allowed by rope length 
int sCommand;                    // calibrated angle command in degrees to servo.write 
int spSail;                      // set point sail angle relative to wind direction 

// Anemometer libraries 
#include <math.h>  

// GPS libraries 
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

// Anemometer Wind Direction Initialization
#define WindSensorPin (2) // The pin location of the anemometer sensor 
int VaneValue;            // raw analog value from wind vane
int CalDirection;         // apparent wind direction: [0,180] clockwise, [0,-179] counterclockwise, 180 is dead zone

// Anemometer Wind Speed Initialization
float WindSpeed;                          // apparent wind speed in knots 
double T = 3.0;                           // time in seconds needed to average readings
unsigned long WindSpeedInterval = T*1000; // time in milliseconds needed to average anemometer readings 
unsigned long previousMillis = 0;         // millis() returns an unsigned long
volatile unsigned long RotationsCounter;  // cup rotation counter used in interrupt routine 
volatile unsigned long ContactBounceTime; // timer to avoid contact bounce in interrupt routine 
uint32_t timer = millis();

//GPS Initialization
TinyGPSPlus tinyGPS; // Create a TinyGPS object
#define GPS_BAUD 9600 // GPS module baud rate
#define ARDUINO_GPS_RX 19 // GPS TX, Arduino RX pin
#define ARDUINO_GPS_TX 18 // GPS RX, Arduino TX pin
SoftwareSerial ssGPS(ARDUINO_GPS_TX, ARDUINO_GPS_RX); // set up a new SoftwareSerial object
float display_timer = 500; // GPS timer loop in milliseconds for displaying data (remove)

  // Set gpsPort to either ssGPS if using SoftwareSerial or Serial1 if using an
  // Arduino with a dedicated hardware serial port
#define gpsPort ssGPS  // Alternatively, use Serial1 on the Leonardo

// Define the serial monitor port. On the Uno, and Leonardo this is 'Serial'
//  on other boards this may be 'SerialUSB'
#define SerialMonitor Serial

// From previous code VVVVVVV (delete later)
// float display_timer = 500;          // GPS timer loop in milliseconds for displaying data 
// int i=0;                            // increment counter for GPS.fix function
// #define GPSECHO  true               //Set GPSECHO to 'false' to turn off echoing the raw GPS data to the Serial console
// // If using software serial, keep this line enabled 
// // (you can change the pin numbers to match your wiring):
// SoftwareSerial mySerial(3,2);       //3,2 originally
// Adafruit_GPS GPS(&mySerial);
// uint32_t timer = millis();
// boolean usingInterrupt=false;

// Sail Servo Prototyping
int getSailServoCommand(int deg);         // calibrate desired sail angle to angle command to servo.Write
      
// Anemometer Wind Direction Prototypes
int getWindDirection(int VaneValue);      // faulty anemometer requires nonlinear equations for calibration

// GPS Prototypes
void printGPSInfo(void);

void setup() {
  Serial.begin(9600); // min baud rate for GPS is 115200 so may have to adjust 

  // Sail Servo Setup
  servoS.attach(7);      // pin for the servoS control
  servoS.write(sOffset); // initialize at zero position 
  
  // Anemometer Wind Direction Setup
  pinMode(WindSensorPin, INPUT); 
  
  // Anemometer Wind Speed Setup 
  attachInterrupt(digitalPinToInterrupt(WindSensorPin), isr_rotation, FALLING); 
  sei(); // Enables interrupts 
  
  //GPS Setup
  gpsPort.begin(GPS_BAUD); // GPS_BAUD currently set to 9600
}

void loop() {
   // Sail Servo Loop 
  if ((h2Delta - h1Delta) < turnAngle){           // if the trajectory turns the boat less than the turnAngle degrees, then maintain 90 degree relative sail angle
    spSail = 90;                                  // set point sail angle relative to wind direction
    sDesired = abs(abs(CalDirection) - spSail);   // CalDirection [-179,180], sail doesn't care about direction   
  }
  else {
    sDesired = sLimit*abs(CalDirection)/180;      // from Davi sail Control Law, executed on turns 
  }
  sCommand = getSailServoCommand(sDesired);       // calibrate desired sail angle to angle command for servo
  servoS.write(sCommand);                         // command sail servo
  h2Delta = h1Delta;                              // current heading becomes previous heading 

  //GPS Loop  
  printGPSInfo(); // print position, altitude, speed

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 500 milliseconds (set in display_timer), print out the current stats
  if (millis() - timer > display_timer) { 
    timer = millis(); // reset the timer
    
    // Anemometer Wind Direction Loop
    VaneValue = analogRead(A4); 
    CalDirection = getWindDirection(VaneValue);
     
    if(CalDirection < -179)         // curve fit has values less than -179 
    CalDirection = -179; 
     
    // Anemometer Wind Speed Loop 
    unsigned long currentMillis = millis(); // current run time
    if ((unsigned long)(currentMillis - previousMillis) >= WindSpeedInterval){  // calculate wind speed after 3 seconds 
      // convert to knots using the formula V = P(2.25/T) / 1.15078 
      WindSpeed = RotationsCounter*2.25/T/1.15078;
      RotationsCounter = 0;                 // reset RotationsCounter after averaging  
      previousMillis = millis();
      }
    
    //Serial.print(VaneValue); Serial.print(",");
    Serial.print(CalDirection); Serial.print(","); 
    Serial.println(WindSpeed); // last output!   
  }

  smartDelay(1000); // "Smart delay" looks for GPS data while the Arduino's not doing anything else

}

// Servo Function definitions
int getSailServoCommand(int sDesired){
  if(sDesired >= sLimit) {  // for desired sail angles greater than the limit, return the limit
    sOffset = sZero - sDesired/12;
    return sLimit + sOffset;
    }
  else {                    // for all other sail angles, return the calibrated sail angle 
    sOffset = sZero - sDesired/12;
    return sDesired + sOffset; 
    }
}

// Anemometer Function Definitions 
// function that the interrupt calls to increment the rotation count 
void isr_rotation () { 
if ((millis() - ContactBounceTime) > 15 ) { // debounce the switch contact.
  RotationsCounter++; 
  ContactBounceTime = millis(); 
  }
  }

// potentiometer values mapped to [-179,180], curve fit from data
int getWindDirection(int VaneValue){     
  if (VaneValue <= 1023 && VaneValue > 683){  // [0,180] degrees
    return round(0.5227*VaneValue - 354.42);  
  }
  else if (VaneValue < 683 && VaneValue >= 0){  // [-179,0] degrees
    return round(0.0004*VaneValue*VaneValue - 0.0005*VaneValue - 188.4);
  } 
  else return 0; 
}

// GPS Function Definitions
void printGPSInfo()
{
  // Print latitude, longitude, speed
  SerialMonitor.print(tinyGPS.location.lat(), 6); SerialMonitor.print(","); 
  SerialMonitor.print(tinyGPS.location.lng(), 6); SerialMonitor.print(","); 
  SerialMonitor.print(tinyGPS.speed.mph()); SerialMonitor.print(","); 
}

// This custom version of delay() ensures that the tinyGPS object
// is being "fed". From the TinyGPS++ examples.
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    // If data has come in from the GPS module
    while (gpsPort.available())
      tinyGPS.encode(gpsPort.read()); // Send it to the encode function
    // tinyGPS.encode(char) continues to "load" the tinGPS object with new
    // data coming in from the GPS module. As full NMEA strings begin to come in
    // the tinyGPS library will be able to start parsing them for pertinent info
  } while (millis() - start < ms);
}
