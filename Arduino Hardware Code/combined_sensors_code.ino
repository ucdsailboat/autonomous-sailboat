//Anemometer libraries 
#include <math.h>  

//GPS libraries 
#include <SoftwareSerial.h>
#include <Adafruit_GPS.h>


//Anemometer Setup
#define Offset 0;         // offset from true magnetic north if global heading desired 
#define WindSensorPin (2) // The pin location of the anemometer sensor 
int VaneValue;            // raw analog value from wind vane 
int Direction;            // translated 0 - 360 direction 
int CalDirection;         // converted value with offset applied z

// Anemometer Wind Speed Init 
int Rotations;                            // displayed onto Serial Monitor 
volatile unsigned long RotationsCounter;  // cup rotation counter used in interrupt routine 
volatile unsigned long ContactBounceTime; // Timer to avoid contact bounce in interrupt routine 
float WindSpeed;                          // speed knots 
double T = 3.0;                           // time in seconds needed to average readings
unsigned long WindSpeedInterval = T*1000; // time in milliseconds needed to average anemometer readings 
unsigned long previousMillis = 0;         // millis() returns an unsigned long

// Anemometer Wind Direction Prototyping
void getHeading(int direction);       // defines ranges for compass rose

//GPS Init
#define GPSECHO  false                //Set GPSECHO to 'false' to turn off echoing the raw GPS data to the Serial console
// If using software serial, keep this line enabled
// (you can change the pin numbers to match your wiring):
//SoftwareSerial mySerial(3, 2);

// If using hardware serial (e.g. Arduino Mega), comment out the
// above SoftwareSerial line, and enable this line instead
// (you can change the Serial number to match your wiring):
HardwareSerial mySerial = Serial1;
Adafruit_GPS GPS(&mySerial);
uint32_t timer = millis();


//GPS prototyping
SIGNAL(TIMER0_COMPA_vect); 
void useInterrupt(boolean v); 

void setup() {
  Serial.begin(9600); // min baud rate for GPS is 115200 so may have to adjust 

  // Anemometer Wind Direction Setup
  pinMode(WindSensorPin, INPUT); 
  attachInterrupt(digitalPinToInterrupt(WindSensorPin), isr_rotation, FALLING); 
  sei(); // Enables interrupts 
  Serial.println("Vane Value\tDirection\tHeading\t\tRotations\tKnots\tSail Servo Angle"); 

  //GPS setup
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

  delay(1000);
  // Ask for firmware version
  mySerial.println(PMTK_Q_RELEASE);
}

void loop() {

  
  //Anemometer Loop
  VaneValue = analogRead(A4); 
  Direction = map(VaneValue, 0, 1023, 0, 360); 
  CalDirection = Direction + Offset; 

  if(CalDirection > 360) 
  CalDirection = CalDirection - 360; 

  if(CalDirection < 0) 
  CalDirection = CalDirection + 360; 
 
  //Wind speed
  unsigned long currentMillis = millis(); // current run time
  if ((unsigned long)(currentMillis - previousMillis) >= WindSpeedInterval){  // calculate wind speed after 3 seconds 
    // convert to knots using the formula V = P(2.25/T) / 1.15078 
    Rotations = RotationsCounter;                
    WindSpeed = RotationsCounter*2.25/T/1.15078;
    RotationsCounter = 0;         // Set Rotations count to 0 ready for calculations 
    previousMillis = millis();
    }
    
    Serial.print(VaneValue); Serial.print("\t\t");
    Serial.print(CalDirection); Serial.print("\t\t"); 
    getHeading(CalDirection); Serial.print("\t\t"); 
    Serial.print(Rotations); Serial.print("\t\t"); 
    Serial.println(WindSpeed); Serial.print("\t\t");





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
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) { 
    timer = millis(); // reset the timer
    
    /*
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
    */
    
    if (GPS.fix) {
      //Serial.print("Location: ");
      //Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      //Serial.print(", "); 
      //Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      Serial.print("Location (in degrees, works with Google Maps): ");
      Serial.print(GPS.latitudeDegrees, 4);
      Serial.print(", "); 
      Serial.print(GPS.longitudeDegrees, 4);
      Serial.print(", "); 
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    }
  }

}

void isr_rotation () { //For anemometer
  if ((millis() - ContactBounceTime) > 15 ) { // debounce the switch contact. 
    RotationsCounter++; 
    ContactBounceTime = millis(); 
  }
}

void getHeading(int direction) { //For anemometer
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
