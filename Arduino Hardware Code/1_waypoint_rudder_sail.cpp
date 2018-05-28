/* Sail and Rudder Controls
Description: The rudder controller is responsible for adjusting the boat orientation
with respect to the defined trajectory. The directions of the compass and the desired path 
are compared. The sail controller optimizes the length of the rope to control the sail based on
the incoming wind angle relative to the boat.
Inputs: desiredPath - angle that the bsailboat needs to travel to reach the waypoint
    heading - actual direction (angle) found via compass
Outputs: angle_rudder - angle of the rudder (type float)
Connections: 
 *  Anemometer: A4 for Wind Direction, Pin 2 for Wind Speed
 *  Sail Servo: Signal to Pin 7
 *  Rudder Servo: Signal to Pin 6
 *  Magnetometer: SDA to SDA, SCL to SCL, Power to Vin, GND to GND
 *  GPS: TX1 is 18, RX1 is 19
Updated by: Jordan Leung, Bryan Zhao, Michele Shi 5/28/18
*/

// rudder controller libraries
#include <StandardCplusplus.h>
// #include <serstream>
// #include <string>
// #include <vector>

//#include <iterator>
//#include <stdio.h>
//#include <stdlib.h>
#include <Servo.h>
#include <math.h>
#include <Wire.h>
//#include <TinyGPS++.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <TinyGPS.h>

// definitions
//#define PI 3.14159265 // (replace with M_PI)
#define RX 4 // on the arduino pins
#define TX 3
#define gpsPort Serial1

// anemometer wind direction 
#define WindSensorPin (2) // pin location of the anemometer sensor 
int VaneValue;            // raw analog value from wind vane
int CalDirection;         // apparent wind direction: [0,180] CW, [0,-179] CCW, 180 is dead zone

TinyGPS gps; // gps object
Servo servoRudder; // declare a servo object
// SoftwareSerial ss(TX, RX); // use pins 4 and 3 for SW serial ports
//using hardware serial 
//HardwareSerial mySerial = Serial1;
//Adafruit_GPS GPS(&mySerial);

// anemometer wind speed
float WindSpeed;                          // apparent wind speed in knots 
double T = 3.0;                           // time in seconds needed to average readings
unsigned long WindSpeedInterval = T*1000; // time in milliseconds needed to average anemometer readings 
unsigned long previousMillis = 0;         // millis() returns an unsigned long
volatile unsigned long RotationsCounter;  // cup rotation counter used in interrupt routine 
volatile unsigned long ContactBounceTime; // timer to avoid contact bounce in interrupt routine 

// Sail Servo Init
Servo servoS;                    // servo object for sail servo 
int turnAngle = 45;              // turn angle in degrees to determine straight sailing or turns
int sDesired = 0;                // desired sail angle relative to nose, uncalibrated  
int sZero = 60;                  // degrees required to zero sail servo in line with nose of boat
int sLimit = 90;                 // constraint angle limit to 90 degrees for the sail, determined by max slack allowed by rope length 
int sOffset;                     // calculated offset from getSailOffset necessary to map servo commands to sail angle (0 to 90 relative to boat nose) 
int sCommand;                    // calibrated angle command in degrees to servo.write 
int spSail;                      // set point sail angle relative to wind direction 

// Anemometer Wind Direction Prototyping
int getWindDirection(int VaneValue);           // faulty anemometer requires nonlinear equations for calibration

// Sail Servo Prototyping
int getSailOffset(int sDesired);               // 0.85*sDesired + 60; used with getSailServoCommand
int getSailServoCommand(int sDesired);         // calibrate desired sail angle to angle command to servo.Write

// assign a unique ID to this sensor at the same time
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);

/* Rudder Controller Init */
int rudOffset = 103;      //offset to zero position (parallel with nose)
// define rudder negative and positive limits 
float rudderPos = 45 + rudOffset; // degrees
float rudderNeg = -45 + rudOffset; // degrees 
float rudderAngle = 0.0; // output angle for the rudder servo
//approximate time of code execution
float sampleTime = 0.2; // sample time of the system (in seconds)
//initialize PID variables
float errorActual = 0; // error = desired angle - actual angle 
float Iaccum = 0;    // accummulator of integrative error.
float Kp = 2;           // proportional gain - starting value suggested by Davi
float Ki = 0.001;       // integral gain - starting value suggsted by Davi
// 0 - north, 90 - east, 180/-180 - south, -90 - west 
// desired path/direction
float desiredPath;
float currHeading;  // current actual angle/direction in degrees
float debugHeading; // [0-360] without saturation
float prevHeading; // store previous heading

// data storage variables
float velocity; // abs. vel of boat [km/h -> NEEDS TO BE CHANGED]
float direction; // current course angle (heading of boat)

// GPS holds positional data
struct location {
  float latitude;
  float longitude;
};

location currentLocation, targetLocation; // define structs with gps coordinates

// vector<location> waypoints; // vector that holds all the different waypoints 

void setup() {
  Serial.begin(9600);
  gpsPort.begin(9600); // needs to be 9600 for proper GPS reading

  // sail controller --
  // Anemometer Wind Direction Setup
  pinMode(WindSensorPin, INPUT); 
  
  // Anemometer Wind Speed Setup 
  attachInterrupt(0, isr_rotation, FALLING); 
  sei(); // Enables interrupts 
  
  // Sail Servo Setup
  servoS.attach(7);      // pin for the servoS control
  servoS.write(sZero); // initialize at zero position 
  
  // rudder controller --
  // define target location (chosen from google earth pro)
  /*targetLocation.latitude = 38.636537; // point 1 at sac aquatic center
  targetLocation.longitude = -121.216988; */
  // targetLocation.latitude = 38.538967; // barony pl. cul de sac
  // targetLocation.longitude = -121.722087;
  targetLocation.latitude =  38.537587; // point 1: lake spafford (closest to shore)
  targetLocation.longitude = -121.748064;
  // Rudder Servo Setup
  servoRudder.attach(6); // attach pin for the rudder servo
  servoRudder.write(rudOffset); // sets initial servo position 
  // For testing purposes (Brisa House)
  //currentLocation.latitude = 38.5836664;
  //currentLocation.longitude = -121.72179470;
  /* Initialise the magentometer sensor */
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Oops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
}

void loop() {
  
  // for rudder ctrl: obtain current heading
  sensors_event_t magEvent; 
  mag.getEvent(&magEvent);
  currHeading = gen_heading(magEvent.magnetic.x, magEvent.magnetic.y); // [-180, 180]
  debugHeading = debug_heading(magEvent.magnetic.x, magEvent.magnetic.y); // for debugging, [0,360] deg from North
  //update current location, number 1 will update currentLocation 
  update_position(currentLocation, 1);
  //gps.f_get_position(&currentLocation.latitude, &currentLocation.longitude);
  // obtain desired direction by calculating path to the waypoint
  desiredPath = calculate_orientation(currentLocation, targetLocation);
  // obtain the needed rudder angle to reduce error b/w desired and actual 
  rudderAngle = rudder_controller(desiredPath, currHeading);
  if (rudderAngle > rudderPos) { rudderAngle = rudderPos; }
  else if (rudderAngle < rudderNeg) {rudderAngle = rudderNeg;}
  servoRudder.write(rudderAngle); // write desired angle into servo 
  
  
  /* for testing purposes */
  //float f_lat, f_lon;
  //Serial.print("Latitude: "); Serial.print(currentLocation.latitude);
  //Serial.print(" Longitude: "); Serial.println(currentLocation.longitude);
  //Serial.print("desiredPath Angle: "); Serial.print(desiredPath); 
  //Serial.print(" Current Heading Angle: "); Serial.print(currHeading);
  //Serial.print(" Rudder Angle: "); Serial.println(rudderAngle-rudOffset);
  // smartdelay(500); // for GPS
  //gps.f_get_position(&f_lat, &f_lon);
  //Serial.print("f_lat = "); Serial.println(f_lat);
  
  // Anemometer Wind Direction Loop
  VaneValue = analogRead(A4); 
  CalDirection = getWindDirection(VaneValue);

  // Curve fit outputs maximum 174 degrees and minimum -172, so set bouds as 180 and -179
  if (CalDirection >= 174)
  CalDirection = 180;
  if (CalDirection <= -172)
  CalDirection = -179;
  
  // Anemometer Wind Speed Loop 
  unsigned long currentMillis = millis(); // current run time
  if ((unsigned long)(currentMillis - previousMillis) >= WindSpeedInterval){  // calculate wind speed after 3 seconds 
    // converts from mph to knots (1 knot = 1.15078 mph) using the formula V = P(2.25/T) / 1.15078 
    WindSpeed = RotationsCounter*2.25/T/1.15078;
    RotationsCounter = 0;                 // reset RotationsCounter after averaging  
    previousMillis = millis();
  }
  
  // Sail Servo Loop 
  if (abs(prevHeading - currHeading) < turnAngle){ // if the trajectory turns the boat less than the turnAngle degrees, then maintain 90 degree relative sail angle
    spSail = 90;                                  // set point sail angle relative to wind direction
    sDesired = abs(abs(CalDirection) - spSail);   // CalDirection [-179,180], sail doesn't care about direction   
  }
  else {
    sDesired = sLimit*abs(CalDirection)/180;      // from Davi sail Control Law, executed on turns 
  }
  sCommand = getSailServoCommand(sDesired);       // calibrate desired sail angle to angle command for servo
  servoS.write(sCommand);                         // command sail servo
  prevHeading = currHeading;                      // current heading becomes previous heading   
  
  //delay(200);
    
  // Serial Monitor Printing Statements 
  Serial.print(currentLocation.latitude); Serial.print(","); 				// GPS: latitude
  Serial.print(currentLocation.longitude); Serial.print(",");               // GPS: longitude 
  Serial.print(gps.speed()*100); Serial.print(",");							// GPS: boat speed in knots 
  Serial.print(CalDirection); Serial.print(","); 						    // Anemometer: Apparent Wind Direction in degrees
  Serial.println(WindSpeed); 											    // Anemometer: Apparent Wind Speed in knots 
} // end of void loop

/// RUDDER CONTROLLER FUNCTIONS ///
// define max and min limits for the boat's orientation (angle)
float saturator(float value) {
  if (value > 180) { 
    value = value - 360;
  }
  if (value < -180) {
    value = value + 360;
  }
  return value;
}

// Rudder Controller Function Definitions 
// define the max and min limits for the rudder angles 
float saturator_rudder(float servoRudderAngle) {
  if (servoRudderAngle > rudderPos) {
    servoRudderAngle = rudderPos;

  }
  if (servoRudderAngle < rudderNeg) {
    servoRudderAngle = rudderNeg;
  }
  return servoRudderAngle;
}

float rudder_controller(float desiredPath, float heading) {
  float angle; // angle of the rudder commanded
  float controlAct = 0; // initialize control action (P + I)

  // desiredPath - angle that the sailboat needs to travel to reach waypoint
  desiredPath = saturator(desiredPath);
  // error between the actual and desired path
  // heading - actual angle found via compass
  errorActual = desiredPath - heading; 
  errorActual = saturator(errorActual);
  // apply PI theory 
  controlAct = P() + I();
  angle = rudOffset + (rudderPos- rudOffset)*(controlAct/180);
 /* // turning the boat in the counterclockwise direction
  if (errorActual < 0) {
    angle = rudOffset + (rudderPos- rudOffset)*(controlAct/180);
    Serial.print(" positive error");
    Serial.print(" angle: "); Serial.println(angle);
  }*/

  angle = saturator_rudder(angle);
  return angle;
}

// proportional control
float P() 
{
  return Kp * errorActual;
}

// integral control, error reduced at each iteration
float I() 
{                                    
  Iaccum = Iaccum + Ki * errorActual * sampleTime; // I = I + Ki* Err* T
    return Iaccum;
}

// generates heading from 0-360 without saturation, no tilt compensation
float debug_heading(float mag_x, float mag_y){
  float debugHeading = (atan2(mag_y,mag_x) * 180) / PI;
  if (debugHeading < 0) debugHeading += 360;
  return debugHeading;
}


// generates current heading in global coordinates from magnetometer values
float gen_heading(float mag_x, float mag_y){
  // convert Gauss units (microteslas) to current compass heading, without tilt compensation
  float currHeading = (atan2(mag_y,mag_x) * 180) / PI; //
  // map the heading so that north is at 0, and the heading goes clockwise
  if (currHeading < -180)
  {
    currHeading = 360 + currHeading;
  }

  // remap to [-180, 180] for the boat heading
  if (currHeading > 180){
    currHeading = currHeading - 360;
  }
  else {
    // keep the currHeading the same, since it's between 0 and 180
  }
  return currHeading;
} // end of function

// update the current position of your boat in GPS
void update_position(struct location local, int number){  // local has latitude and longitude
  bool newData = false; // for debugging, determine whether GPS got new data
  unsigned long startTime = millis(); // start the clock
  unsigned long delayThresh = 50; // delay threshold [ms]
  // obtain readings from the GPS to the serial port in 1/2 second loop
  do
  {
    // while(ss.available()) // get number of bytes (characters) available for reading from serial port
    while(gpsPort.available())
    {
      // char raw_data = ss.read(); // reads incoming serial data buffer, used from example code
      // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      // if(gps.encode(ss.read())) // feed the characters from your GPS, did a new valid sentence come in?
      if(gps.encode(gpsPort.read()))
      {
        // Get the position, where latitude and longitude are variables
        // of float type, and the actual values are returned. Float types 
        // are easier to use, but result in larger and slower code. The age 
        // variable must be unsigned long type.
        gps.f_get_position(&local.latitude, &local.longitude); // set latitude and longitude of 'local' variable
        
        if (number == 1) {
          // stores local coordinates into currentLocation coordinates
          currentLocation.latitude = local.latitude;
          currentLocation.longitude = local.longitude;
        }

        direction = gps.f_course(); // obtain the course (direction), defined in main code, for saving data
        velocity = gps.f_speed_kmph(); // obtain velocity in km/h (can change), for saving data
        newData = true; // for debugging
      }
    }
  } while (millis() - startTime < delayThresh); // incorporate a smart delay (ref. below)
} // end of function

// calculates the linear distance between two waypoints
int calculate_distance(struct location point_1, struct location point_2){
  return (int)TinyGPS::distance_between(point_1.latitude, point_1.longitude, point_2.latitude, point_2.longitude);
}

// calculates the heading orientation that you need to get from point 1 to point 2, with constraints of [-180,180]
float calculate_orientation(struct location point_1, struct location point_2){
  // generate the course angle, which is measured clockwise from north, between point 1 and 2 -- since
  // the course angle will be used for the desired heading of the boat
  float boatCourse = 0.0;
  float courseAngle = TinyGPS::course_to(point_1.latitude, point_1.longitude, point_2.latitude, point_2.longitude);
  // map the limits to [-180, 180] for the boat's frame of reference (heading)
  // e.g. if courseAngle is 200 deg referenced from north, relative angle is mapped to -160
  if(courseAngle > 180){
    boatCourse = courseAngle-360;
  }
  // e.g. if courseAngle is -100 deg referenced from north, relative angle is mapped to +160
  if(courseAngle < -180){
    boatCourse = courseAngle+360;
  }
  return boatCourse;
} // end of function

// incorporate delay for the GPS to read all incoming data to serial port
/* static void smartdelay(unsigned long ms)
{
  unsigned long startTime = millis();
  do 
  {
    while (gpsPort.available()) // make sure 'Serial3' is mapped to the right serial output or rename it
      gps.encode(ss.read());
  } while (millis() - startTime < ms);
} */ // uncomment once you have software serial

/// SAIL CONTROLLER FUNCTIONS ///

// Servo Function definitions
int getSailOffset(int sDesired){            // used in getSailServoCommand function 
  return sDesired*0.85 + sZero;             // curve fit of data 
}

int getSailServoCommand(int sDesired){
  if(sDesired >= sLimit) {  // for desired sail angles greater than the limit, return the limit
    sCommand = getSailOffset(sLimit);
    return round(sCommand);
    }
  else {                    // for all other sail angles, return the calibrated sail angle 
    sCommand = getSailOffset(sDesired);
    return round(sCommand); 
    }
}

// Anemometer function definitions 
// function that the interrupt calls to increment the rotation count 
void isr_rotation () { 
if ((millis() - ContactBounceTime) > 15 ) { // debounce the switch contact.
  RotationsCounter++; 
  ContactBounceTime = millis(); 
  }
  }

// potentiometer values mapped to [-179,180], curve fit from data
int getWindDirection(int VaneValue){     
  return 0.3387*VaneValue - 172.4;    // obtained from curve fit data 
}
