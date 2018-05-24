/* Rudder Control 
Description: The rudder controller is responsible for adjusting the boat orientation
with respect to the defined trajectory. The directions of the compass and the desired path 
are compared 
Inputs: desiredPath - angle that the bsailboat needs to travel to reach the waypoint
    heading - actual direction (angle) found via compass
Outputs: angle_rudder - angle of the rudder (type float)
Updated by: Jordan Leung & Bryan Zhao 5/21/18
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
// GPS libraries 

//#include <SoftwareSerial.h>

// definitions
//#define PI 3.14159265 // (replace with M_PI)
#define RX 4 // on the arduino pins
#define TX 3
#define gpsPort Serial1

TinyGPS gps; // gps object
Servo servoRudder; // declare a servo object
// SoftwareSerial ss(TX, RX); // use pins 4 and 3 for SW serial ports
//using hardware serial 
//HardwareSerial mySerial = Serial1;
//Adafruit_GPS GPS(&mySerial);

// assign a unique ID to this sensor at the same time
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);

/* Rudder Controller Init */
int rudOffset = 91;      //offset to zero position (parallel with nose)
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
float debugHeading;

// data storage variables
float velocity; // abs. vel of boat [km/h -> NEEDS TO BE CHANGED]
float direction; // current course angle (heading of boat)

// GPS holds positional data
struct location {
  float latitude;
  float longitude;
};

location currentLocation, targetLocation; // define structs with gps coordinates

vector<location> waypoints; // vector that holds all the different waypoints 

void setup() {
  Serial.begin(115200);
  gpsPort.begin(115200);

  // define target location (chosen from google earth pro)
  targetLocation.latitude = 38.538967; // barony pl. cul de sac
  targetLocation.longitude = -121.722087;
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
  // obtain current heading
  sensors_event_t magEvent; 
  mag.getEvent(&magEvent);
  currHeading = gen_heading(magEvent.magnetic.x, magEvent.magnetic.y);
  debugHeading = debug_heading(magEvent.magnetic.x, magEvent.magnetic.y); // for debugging, 0-360 deg from North
  //update current location, number 1 will update currentLocation 
  update_position(currentLocation, 1);
  //gps.f_get_position(&currentLocation.latitude, &currentLocation.longitude);
  // obtain desired direction by calculating path to the waypoint
  desiredPath = calculate_orientation(currentLocation, targetLocation);
  // obtain the needed rudder angle to reduce error b/w desired and actual 
  rudderAngle = rudder_controller(desiredPath, currHeading);
  servoRudder.write(rudderAngle); // write desired angle into servo 
  
  
  /* for testing purposes */
  //float f_lat, f_lon;
  Serial.print("Latitude: "); Serial.print(currentLocation.latitude);
  Serial.print(" Longitude: "); Serial.println(currentLocation.longitude);
  Serial.print("desiredPath Angle: "); Serial.print(desiredPath); 
  Serial.print(" Current Heading Angle: "); Serial.print(currHeading);
  Serial.print(" Rudder Angle: "); Serial.println(rudderAngle-rudOffset);
  // smartdelay(500); // for GPS
  //gps.f_get_position(&f_lat, &f_lon);
  //Serial.print("f_lat = "); Serial.println(f_lat);
  delay(200);
}


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
  // turning the boat in the clockwise direction 
  if (errorActual > 0) {
    angle = rudOffset + (rudderPos- rudOffset)*(controlAct/180);
  }
  // turning the boat in the counterclockwise direction
  else {
    angle = rudOffset + (rudderNeg- rudOffset)*(controlAct/180);
  }

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
  unsigned long delayThresh = 300; // delay threshold [ms]
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
