#include<Wire.h>
#include<Zumo32U4.h> //Include the needed libraries

Zumo32U4Motors motors;
Zumo32U4Encoders encoders;    //initialize the different Zumo-functionalities
Zumo32U4ButtonA buttonA;



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);  // Start the serial-communication between the robot and the computer.
  buttonA.waitForPress(); //Starts the program after a click on button A and a small delay
  delay(100);
}

void loop() {
  // put your main code here, to run repeatedly:
   
}
