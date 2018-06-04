/* Anemometer and GPS Combined Code - Modified 5/13 M. Shi  
 * Anemometer loop is currently in the GPS timer loops that displays every 500ms (set in display_timer variable)
 * Anemometer readings averaged every 3000ms (set in WindSpeedInterval variable)
*/ 

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
float display_timer = 500; // GPS timer loop in milliseconds for displaying data (remove)

  // Set gpsPort to either ssGPS if using SoftwareSerial or Serial1 if using an
  // Arduino with a dedicated hardware serial port
#define gpsPort Serial1

// Define the serial monitor port. On the Uno, and Leonardo this is 'Serial'
//  on other boards this may be 'SerialUSB'
//#define SerialMonitor Serial

// Anemometer Wind Direction Prototypes
int getWindDirection(int VaneValue);      // faulty anemometer requires nonlinear equations for calibration

// GPS Prototypes
void printGPSInfo(void);

void setup() {
  Serial.begin(9600); // min baud rate for GPS is 115200 so may have to adjust 
  
  // Anemometer Wind Direction Setup
  pinMode(WindSensorPin, INPUT); 
  
  // Anemometer Wind Speed Setup 
  attachInterrupt(0, isr_rotation, FALLING); 
  sei(); // Enables interrupts 
  
  //GPS Setup
  gpsPort.begin(GPS_BAUD); // GPS_BAUD currently set to 9600
}

void loop() {
 
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
  Serial.print(tinyGPS.location.lat(), 6); Serial.print(","); 
  Serial.print(tinyGPS.location.lng(), 6); Serial.print(","); 
  Serial.print(tinyGPS.speed.mph()); Serial.print(","); 
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
