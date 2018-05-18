// EME185, UC Davis Senior Design Project

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* This driver reads raw data from the BNO055 sensor
 *  Connections
 *  ----------
 *  Connect SCL to Analog 5 for Arduino Uno
 *  Connect SDA to Analog 4 " "
 *  Connect VDD to 3.3V DC
 *  Connect GND to common ground
 */
 
/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

// declare vars 
Adafruit_BNO055 bno = Adafruit_BNO055(55); // declare bno object
float heading;
float getHeading(float magVals); // convert microteslas to boat heading

// setup 
void setup(void) 
{
  Serial.begin(9600);
  Serial.println("Orientation Sensor Test"); Serial.println("");
  
  /* initialise the sensor */
  if(!bno.begin())
  {
    /* there was a problem detecting the BNO055 ... check your connections */
    Serial.print("Oops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  delay(1000); // wait for a second
  bno.setExtCrystalUse(true); // use external crystal
}

void loop(void) 
{
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  // imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER); // uT
  
  /* Display magnetometer (microtesla) */ 
  /* Serial.print("[MAG]");
  Serial.print("\tX: ");
  Serial.print(mag.x());
  Serial.print("\tY: ");
  Serial.print(mag.y());
  Serial.print("\tZ: ");
  Serial.println(mag.z()); 

  heading = getHeading(mag.x(), mag.y(), mag.z()); // acquire heading from magnetometer output
  Serial.print("Heading [deg]: ");
  Serial.println(heading); */
  
 imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER); // degrees
  
  /* Display Euler orientation (degrees) */
  Serial.print("[O]");
  Serial.print("\tX: "); // yaw
  Serial.print(euler.x());
  Serial.print("\tY: "); // roll
  Serial.print(euler.y());
  Serial.print("\tZ: "); // pitch
  Serial.println(euler.z());

  // imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    
  ///* Display the angular velocity (rad/s) */
  //Serial.print("[GYRO]");
  //Serial.print("\tX: ");
  //Serial.print(gyro.x());
  //Serial.print("\tY: ");
  //Serial.print(gyro.y());
  //Serial.print("\tZ: ");
  //Serial.println(gyro.z());
  //
  // imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  //  
  ///* Display the acceleration (m/s^2)*/
  //Serial.print("[ACCEL]");
  //Serial.print("\tX: ");
  //Serial.print(accel.x());
  //Serial.print("\tY: ");
  //Serial.print(accel.y());
  //Serial.print("\tZ: ");
  //Serial.println(accel.z());
  //
  // imu::Vector<3> linaccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  //  
  ///* Display the linear acceleration (m/s^2) */
  //Serial.print("[LIN]");
  //Serial.print("\tX: ");
  //Serial.print(linaccel.x());
  //Serial.print("\tY: ");
  //Serial.print(linaccel.y());
  //Serial.print("\tZ: ");
  //Serial.println(linaccel.z());

  /* Display calibration status for each sensor. */
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.print("CALIBRATION: Sys=");
  Serial.print(system, DEC);
  Serial.print(" Gyro=");
  Serial.print(gyro, DEC);
  Serial.print(" Accel=");
  Serial.print(accel, DEC);
  Serial.print(" Mag=");
  Serial.println(mag, DEC);

  delay(BNO055_SAMPLERATE_DELAY_MS);

  Serial.println(); // end of script!
}

// compute heading from microtesla output of magnetometer, assume no tilt
float getHeading(float magX, float magY, float magZ)
{
    heading = atan2(magX, magY) * 180/PI; // heading in degrees
    return (heading);
}
