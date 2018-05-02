#include <math.h> 

#define Offset 0; 
#define WindSensorPin (2) // The pin location of the anemometer sensor 

int VaneValue;// raw analog value from wind vane 
int Direction;// translated 0 - 360 direction 
int CalDirection;// converted value with offset applied z
int LastValue; 
volatile unsigned long Rotations; // cup rotation counter used in interrupt routine 
volatile unsigned long ContactBounceTime; // Timer to avoid contact bounce in interrupt routine 
float WindSpeed; // speed miles per hour 
double T=3.0;

void setup() { 
LastValue = 1; 
Serial.begin(9600);
pinMode(WindSensorPin, INPUT); 
attachInterrupt(digitalPinToInterrupt(WindSensorPin), isr_rotation, FALLING); 

Serial.println("Vane Value\tDirection\tHeading\t\tRotations\tMPH"); 
} 

void loop() { 
//Wind direction  
VaneValue = analogRead(A4); 
Direction = map(VaneValue, 0, 1023, 0, 360); 
CalDirection = Direction + Offset; 

if(CalDirection > 360) 
CalDirection = CalDirection - 360; 

if(CalDirection < 0) 
CalDirection = CalDirection + 360; 


//Wind speed
Rotations = 0; // Set Rotations count to 0 ready for calculations 

sei(); // Enables interrupts 
delay (3000); // Wait 3 seconds to average 
cli(); // Disable interrupts 

// convert to mp/h using the formula V=P(2.25/T) 
// V = P*(2.25/T) 
 
LastValue = CalDirection; 
WindSpeed = Rotations*2.25/T;
 
Serial.print(VaneValue); Serial.print("\t\t"); 
Serial.print(CalDirection); Serial.print("\t\t"); 
getHeading(CalDirection); Serial.print("\t\t"); 
Serial.print(Rotations); Serial.print("\t\t"); 
Serial.println(WindSpeed); 
} 

// This is the function that the interrupt calls to increment the rotation count 
void isr_rotation () { 

if ((millis() - ContactBounceTime) > 15 ) { // debounce the switch contact. 
Rotations++; 
ContactBounceTime = millis(); 
}
}

// Converts compass direction to heading 
void getHeading(int direction) { 
if(direction < 22) 
Serial.print("N"); 
else if (direction < 67) 
Serial.print("NE"); 
else if (direction < 112) 
Serial.print("E"); 
else if (direction < 157) 
Serial.print("SE"); 
else if (direction < 212) 
Serial.print("S"); 
else if (direction < 247) 
Serial.print("SW"); 
else if (direction < 292) 
Serial.print("W"); 
else if (direction < 337) 
Serial.print("NW"); 
else 
Serial.print("N"); 
}
