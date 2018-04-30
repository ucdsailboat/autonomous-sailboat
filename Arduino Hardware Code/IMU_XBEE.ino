#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
  
Adafruit_BNO055 bno = Adafruit_BNO055(55);
int t =60;
int t2=100;

void setup(void) 
{
  Serial2.begin(19200);
  Serial.begin(19200);
  Serial.println("Orientation Sensor Test"); 
  Serial.println("");
  
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
  
/* Display magnetometer (microtesla) */
Serial2.print("[MAG]");delay(t);
Serial2.print("\tX: ");delay(t);
Serial2.print(mag.x());delay(t);
Serial2.print("\tY: ");delay(t);
Serial2.print(mag.y());delay(t);
Serial2.print("\tZ: ");delay(t);
Serial2.print(mag.z());delay(t);
Serial2.print("\n");delay(t2);

 imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  
// Display Euler orientation (degrees) 
Serial2.print("[O]");delay(t);
Serial2.print("\tX: ");delay(t);
Serial2.print(euler.x());delay(t);
Serial2.print("\tY: ");delay(t);
Serial2.print(euler.y());delay(t);
Serial2.print("\tZ: ");delay(t);
Serial2.print(euler.z());delay(t);
Serial2.print("\n");delay(t2);
/*
 imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

// Display the angular velocity (rad/s) 
Serial2.print("[GYRO]");delay(t);
Serial2.print("\tX: ");delay(t);
Serial2.print(gyro.x());delay(t);
Serial2.print("\tY: ");delay(t);
Serial2.print(gyro.y());delay(t);
Serial2.print("\tZ: ");delay(t);
Serial2.print(gyro.z());delay(t);
Serial2.print("\n");delay(t2);

 imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  
// Display the acceleration (m/s^2)
Serial2.print("[ACCEL]");delay(t);
Serial2.print("\tX: ");delay(t);
Serial2.print(accel.x());delay(t);
Serial2.print("\tY: ");delay(t);
Serial2.print(accel.y());delay(t);
Serial2.print("\tZ: ");delay(t);
Serial2.print(accel.z());delay(t);
Serial2.print("\n");delay(t2);

 imu::Vector<3> linaccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  
// Display the linear acceleration (m/s^2)
Serial2.print("[LIN]");delay(t);
Serial2.print("\tX: ");delay(t);
Serial2.print(linaccel.x());delay(t);
Serial2.print("\tY: ");delay(t);
Serial2.print(linaccel.y());delay(t);
Serial2.print("\tZ: ");delay(t);
Serial2.print(linaccel.z());delay(t);
Serial2.print("\n");//delay(t2);
*/
  if (Serial2.available())
  { // If data comes in from XBee, send it out to serial monitor
    Serial.write(Serial2.read());
    if (Serial2.read()=='\n')
    {
    Serial.println();
    }
  }
  //delay(t2);
}
