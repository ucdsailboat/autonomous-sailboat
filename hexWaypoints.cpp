// 
#include <StandardCplusplus.h>
#include <serstream>
#include <string>
#include <vector>
#include <iterator>
#include <inttypes.h>
#include <Servo.h>
#include <math.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <Math.h>
#include <Wire.h>
#include <stdio.h>
#include <stdlib.h>
#include <SPI.h>
#include <SD.h>

File dataFile;

#define PI 3.14159265

using namespace std;

struct location {
	float latitude;
	float longitude;
};

TinyGPS gps;
int cont_debug = 0;
float start, tempo;
float distanciaInitial, distanceTraveled, lastDistanceDest = 0;

location startLocation, currentLocation, lastLocation, nextLocation, targetLocation;

vector<location> hexWaypoints;

void setup()
{
		// push all waypoints, custom selected from Google Earth (hexagon of waypoints)

	  //			 3  ____	4				
		//				 /  	\
		//			2  \____/  5
		//				 1    6

		targetLocation.latitude = 38.636537; // point 1
		targetLocation.longitude = -121.216988;
		hexWaypoints.push_back(targetLocation);

		targetLocation.latitude = 38.636631; // point 2
		targetLocation.longitude = -121.217336;
		hexWaypoints.push_back(targetLocation);

		targetLocation.latitude = 38.636931; // point 3
		targetLocation.longitude = -121.217237;
		hexWaypoints.push_back(targetLocation); 

		targetLocation.latitude = 38.637071; // point 4
		targetLocation.longitude = -121.216845;
		hexWaypoints.push_back(targetLocation); 

		targetLocation.latitude = 38.636928; // point 5
		targetLocation.longitude = -121.216509;
		hexWaypoints.push_back(targetLocation); 

		targetLocation.latitude = 38.636537; // point 6, connects back to point 1
		targetLocation.longitude = -121.216605;
		hexWaypoints.push_back(targetLocation);

		Serial.begin(9600);
		startTime = millis();

}

void loop(){} // empty code