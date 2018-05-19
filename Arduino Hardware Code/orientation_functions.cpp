// Functions that are utilized in the sailboat main script

void update_position(struct location local){  // local has latitude and longitude
  unsigned long startTime = millis(); // start the clock
  unsigned long delayThresh = 500; // delay threshold [ms]
  // obtain readings from the GPS to the serial port in 1/2 second loop
  do
  {
    while(Serial3.available()) // get number of bytes (characters) available for reading from serial port
    {
      int raw_data = Serial3.read(); // reads incoming serial data buffer, used from example code
      if(gps.encode(raw_data)) // feed the characters from your GPS
      {
        // Get the position, where latitude and longitude are variables
        // of float type, and the actual values are returned. Float types 
        // are easier to use, but result in larger and slower code. The age 
        // variable must be unsigned long type.
        gps.f_get_position(&local.latitude, &local.longitude); // set latitude and longitude of 'local' variable
        direction = gps.f_course(); // obtain the course (direction), defined in main code
        velocity = gps.f_speed_kmph(); // obtain velocity in km/h (can change)
      }
    }
  } while (millis() - startTime < delayThresh); // incorporate a smart delay (ref. below)
} // end of function

// calculates the linear distance between two waypoints
int calculate_distance(struct location point_1, struct location point_2){
  return (int)TinyGPS::distance_between(point_1.latitude, point_1.longitude, point_2.latitude, point_2.longitude);
}

// calculates the heading orientation that you need to get from point 1 to point 2,
// with constraints of [-180,180]
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

// incorporate delay for the GPS to read all the data necessary
// (this is already integrated into update_position function)
static void smartdelay(unsigned long ms)
{
  unsigned long startTime = millis();
  do 
  {
    while (Serial3.available())
      gps.encode(Serial3.read());
  } while (millis() - startTime < ms);
}