#include <math.h>
#include <TimerOne.h>


float origin[2] = {38.638461, -121.214811}; //GPS coordinates of origin (latitude, longitude)
float curLocation[2]={origin[0],origin[1]};  //GPS coordinates of current location
float earthR= 6378100;      //m
int windGlobalAngle=0;      //(-180,180] global angle of wind from north
volatile float a;                    //used for calculating haversine angle
int againstWindAngle=45;  

float windSpeed = 1;      //m/s
float windAngle = 0;      //angle of wind in body frame (-180,180]
volatile float desiredAngle =45;

float rLimit = 50;      //m
volatile float radius;  //current distance from origin

boolean left = 1;       //should go over left(1) or right(0) shoulder next time
boolean turning=0;
int turningBuffer=0;    //loop counter to allow boat to get back into boundary

TimerOne T1;
int T = 500;              //ms between radius calcs

void turn(double curLocation[]);

void setup() {
  Serial.begin(9600);
  //sei();                //enable interrupts. necessary for this library defined interrupt?
  //T1.initialize(T);
  //T1.attachInterrupt(turn, 1000); 
}

void loop() {
  curLocation[0]=curLocation[0]+windSpeed*sin(M_PI/180*(windGlobalAngle+windAngle))/earthR*360;   //Where going N is (+) and S is (-)
  curLocation[1]=curLocation[1]+windSpeed*cos(M_PI/180*(windGlobalAngle+windAngle))/earthR*360;   //Where going W is (+) and E is (-)
  //Serial.print(windSpeed*sin(M_PI/180*(windGlobalAngle+windAngle))/earthR*360,8);
  //Serial.print("\t");
  Serial.print(curLocation[0],8);
  Serial.print(",");
  Serial.print(curLocation[1],8);
  Serial.print(",");
  Serial.println(radius-rLimit,8);
  
  a = haversin((origin[0] - curLocation[0])*M_PI/180) + cos(origin[0]*M_PI/180)*cos(curLocation[0]*M_PI/180)*haversin((origin[1]- curLocation[1])*M_PI/180);
  radius=2*earthR*atan2(sqrt(a),sqrt(1-a));
  /*Serial.print(a,6);
  Serial.print("\t");
  Serial.print(radius, 6);
  Serial.print("\t");
  Serial.println(windAngle, 6);*/
  
  //Change directions if beyond the boundary
  if (!turning){
    if (radius > rLimit && abs(desiredAngle) == againstWindAngle) {
      desiredAngle = 180;
      turning=1;
    }
    else if(radius > rLimit && desiredAngle == 180 && left) {
      desiredAngle = -againstWindAngle;
      left=!left;
      turning=1;
    }
    else if(radius > rLimit && desiredAngle == 180 && !left) {
      desiredAngle = againstWindAngle;
      left=!left;
      turning=1;
    }
  }
  else {
    turningBuffer++;
    if (turningBuffer>5){
      turningBuffer=0;
      turning=0;          
    }
  }
  
  if (desiredAngle!= windAngle){
    windAngle=desiredAngle;
  }
//  windGlobalAngle++;
delay(100);
}

float haversin(float angle){                //radians
    return pow(sin(angle/2),2);
}

void turn(float curLocation[2]) {
  //Use Haversine formula to calculate distance between two coordinates
  float a = 2*earthR*asin(sqrt(haversin(origin[0] - curLocation[0]) + cos(origin[0])*cos(curLocation[0])*haversin(origin[1] - curLocation[1])));

  //Change directions if beyond the boundary
  if (radius > rLimit && desiredAngle == againstWindAngle) {
    desiredAngle = 180;
  }
  else if(radius > rLimit && desiredAngle == 180 && windAngle>0) {
    desiredAngle = -againstWindAngle;
  }
  else if(radius > rLimit && desiredAngle == 180 && windAngle<0) {
    desiredAngle = againstWindAngle;
  }
}         

