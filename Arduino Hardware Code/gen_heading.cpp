// generates current heading of the boat, within the constraints of
// [-180, 180] degrees. 
//
// input: float event.magnetic.y, float event.magnetic.x
// output: float heading [degrees] 
//
// written by Bryan Z. 5/18/18

// before void setup() in main code, the following is needed:
//  Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);
// in the void loop():
//  sensors_event_t event; 
//  mag.getEvent(&event);

// generates current heading in global coordinates
float gen_heading(float mag_x, float mag_y){
  // convert Gauss units (microteslas) to current compass heading, without tilt compensation
  float currHeading = (atan2(mag_y,mag_x) * 180) / PI; //
  // map the heading so that north is at 0, and the heading goes clockwise
  if (currHeading < 0)
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
