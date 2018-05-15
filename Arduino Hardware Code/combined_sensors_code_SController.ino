/* Anemometer and GPS Combined Code - Modified 5/13 M. Shi  
 * Anemometer loop is currently in the GPS timer loops that displays every 500ms (set in display_timer variable)
 * Anemometer readings averaged every 3000ms (set in WindSpeedInterval variable)
*/ 

// Servo libraries 
#include <Servo.h> 

// assumed from rudder controller, from joystick for testing 
int h1Delta = 0;          // current heading relative to boat (0 to 180 clockwise, 0 to -179 counterclockwise)
int h2Delta = 0;          // previous heading relative to boat 

// Anemometer libraries 
#include <math.h>  

// GPS libraries 
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

// Sail Servo Init
Servo servoS;                    // servo object for sail servo 
int turnAngle = 45;              // turn angle in degrees to determine straight sailing or turns
int sDesired = 0;                // desired sail angle relative to nose, uncalibrated  
int sZero = 40;                  // degrees required to zero sail servo in line with nose of boat
int sOffset = sZero - sDesired/12;  // offset necessary to map servo commands to sail angle (0 to 90 relative to boat nose)  
int sLimit = 90;                 // constraint angle limit to 90 degrees for the sail, determined by max slack allowed by rope length 
int sCommand;                    // calibrated angle command in degrees to servo.write 
int spSail;                      // set point sail angle relative to wind direction 

// Anemometer Wind Direction Init  
#define WindSensorPin (2) // The pin location of the anemometer sensor 
int VaneValue;            // raw analog value from wind vane
int CalDirection;         // apparent wind direction: [0,180] clockwise, [0,-179] counterclockwise, 180 is dead zone

// Sail Servo Prototyping
int getSailServoCommand(int deg);         // calibrate desired sail angle to angle command to servo.Write

// Anemometer Wind Speed Init 
float WindSpeed;                          // apparent wind speed in knots 
double T = 3.0;                           // time in seconds needed to average readings
unsigned long WindSpeedInterval = T*1000; // time in milliseconds needed to average anemometer readings 
unsigned long previousMillis = 0;         // millis() returns an unsigned long
volatile unsigned long RotationsCounter;  // cup rotation counter used in interrupt routine 
volatile unsigned long ContactBounceTime; // timer to avoid contact bounce in interrupt routine 

//GPS Init
float display_timer = 500;          // GPS timer loop in milliseconds for displaying data 
int i=0;                            // increment counter for GPS.fix function
#define GPSECHO  true               //Set GPSECHO to 'false' to turn off echoing the raw GPS data to the Serial console
// If using software serial, keep this line enabled 
// (you can change the pin numbers to match your wiring):
SoftwareSerial mySerial(3,2);       //3,2 originally

Adafruit_GPS GPS(&mySerial);
uint32_t timer = millis();
boolean usingInterrupt=false;
      
// Anemometer Wind Direction Prototyping
int getWindDirection(int VaneValue);      // faulty anemometer requires nonlinear equations for calibration

// GPS Prototyping
SIGNAL(TIMER0_COMPA_vect);      // Interrupt is called once a millisecond, looks for any new GPS data, and stores it
void useInterrupt(boolean);     //GPS interrupt

void setup() {
  Serial.begin(115200); // min baud rate for GPS is 115200 so may have to adjust 

  // Sail Servo Setup
  servoS.attach(7);      // pin for the servoS control
  servoS.write(sOffset); // initialize at zero position 
  
  // Anemometer Wind Direction Setup
  pinMode(WindSensorPin, INPUT); 
  
  // Anemometer Wind Speed Setup 
  attachInterrupt(digitalPinToInterrupt(WindSensorPin), isr_rotation, FALLING); 
  sei(); // Enables interrupts 
  
  //GPS Setup
  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!
  useInterrupt(true);
 
  // Ask for firmware version
  mySerial.println(PMTK_Q_RELEASE);
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
      // in case you are not using the interrupt above, you'll
  // need to 'hand query' the GPS, not suggested :(
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c) Serial.print(c);
  }
  
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

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
    Serial.print(CalDirection); Serial.print(" "); 
    Serial.print(WindSpeed); Serial.print(",");
    
    if (!(GPS.fix)){
      Serial.print(0.00000000, 8);     //These still work with Google Maps
      Serial.print(","); 
      Serial.print(0.0000000000, 8);
      Serial.print(","); 
      Serial.println(GPS.speed);        //knots
      i++;
    }
    if (GPS.fix) { 
      Serial.print(GPS.latitudeDegrees, 8);     //These still work with Google Maps
      Serial.print(","); 
      Serial.print(GPS.longitudeDegrees, 8);
      Serial.print(","); 
      Serial.println(GPS.speed);                //knots
      i++; 
    }
  }

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
SIGNAL(TIMER0_COMPA_vect) { // Interrupt is called once a millisecond, looks for any new GPS data, and stores it
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#endif
}

void useInterrupt(boolean v) { //GPS interrupt
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}


/* Notes: Extra GPS Printing Commands 
    Serial.print("\nTime: ");
    Serial.print(GPS.hour, DEC); Serial.print(':');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality); 

    if GPS.fix is true
    //Serial.print("Location: ");
    //Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
    //Serial.print(","); 
    //Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
    //Serial.print("Location (in degrees, works with Google Maps): ");

    //Serial.print(","); 
    //Serial.print(GPS.speed);        //knots
    //Serial.print(","); 
    //Serial.print(GPS.angle);
    //Serial.print("Altitude: "); 
    //Serial.print(GPS.altitude);
    //Serial.print(","); 
    //Serial.println((int)GPS.satellites);
    */
  
