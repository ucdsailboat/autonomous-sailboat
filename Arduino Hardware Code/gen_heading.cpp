// Generates current heading of the boat, within the constraints of
// [-180, 180] degrees. 
/* 
Input: float event.magnetic.y, float event.magnetic.x
Output: float heading [degrees] 
*/
// Written by Bryan Z. 5/18/18

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