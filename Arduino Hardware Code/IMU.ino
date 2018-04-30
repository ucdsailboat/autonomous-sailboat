#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
  
Adafruit_BNO055 bno = Adafruit_BNO055(55);

void setup(void) 
{
  Serial.begin(9600);
  Serial.println("Orientation Sensor Test"); Serial.println("");
  
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  delay(1000);
    
  bno.setExtCrystalUse(true);
}

void loop(void) 
{
 imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  
///* Display magnetometer (microtesla) */
//Serial.print("[MAG]");
//Serial.print("\tX: ");
//Serial.print(mag.x());
//Serial.print("\tY: ");
//Serial.print(mag.y());
//Serial.print("\tZ: ");
//Serial.println(mag.z());
//
 imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  
/* Display Euler orientation (degrees) */
Serial.print("[O]");
Serial.print("\tX: ");
Serial.print(euler.x());
Serial.print("\tY: ");
Serial.print(euler.y());
Serial.print("\tZ: ");
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

Serial.println();

  delay(100);
}
