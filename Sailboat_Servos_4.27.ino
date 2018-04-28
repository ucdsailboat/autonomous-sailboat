

String readString;
#include <Servo.h> 
Servo myservoa; // creates servo object to control sail servo 
Servo myservob; // creates servo object to control rudder servo

void setup() {
  Serial.begin(9600);

  myservoa.attach(6);  //the pin for the servoa control
  myservob.attach(7);  //the pin for the servob control
  Serial.println("Takes inputs like 90a,0b, for respective servos"); // input prompt need comma after each command
}

void loop() {

  if (Serial.available())  {
    char c = Serial.read();  //gets one byte from serial buffer
    if (c == ',') {
      if (readString.length() >1) {
        Serial.println(readString); //prints string to serial port out

        int n = readString.toInt();  //convert readString into a number

        // auto select appropriate value, copied from someone elses code.
        if(n >= 500)
        
 

          Serial.print("writing Angle: ");
          Serial.println(n+40);
          if(readString.indexOf('a') >0) myservoa.write(n+40);
          if(readString.indexOf('b') >0) myservob.write((40-n/12)+n);
        
         readString=""; //clears variable for new input
      }
    }  
    else {     
      readString += c; //makes the string readString
    }
  }
}


