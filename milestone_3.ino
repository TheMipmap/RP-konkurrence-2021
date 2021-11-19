#include <Zumo32U4.h>

#include <Wire.h>
#include <Zumo32U4.h>
Zumo32U4Buzzer buzzer;
#include <Wire.h>
#include <Zumo32U4.h>
Zumo32U4LCD LCD;
Zumo32U4Encoders encoders;
Zumo32U4ButtonA A;
Zumo32U4ButtonB B;
Zumo32U4ButtonC C;
Zumo32U4Motors motors;
Zumo32U4LineSensors lineSensors;
#define NUM_SENSORS 5

int speedselect;

int small;

bool useEmitters = true;


struct LineSensorsWhite { // True if White, False if Black
bool L; 
bool LC; 
bool C; 
bool RC; 
bool R;
};

LineSensorsWhite sensorsState = {0,0,0,0,0};
int lineSensorValues[NUM_SENSORS];
int threshold =200;



void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
lineSensors.initFiveSensors(); 
}



void loop() {
moveUntilLine();                  // her køre zumo indtil at den finder en hvid linje
delay(1000);
readSensors(sensorsState);        // her læser den sensorsstate til at derefter print dens values til serilamonitor
printSensorValues();
}



void drive0(int speedselect){      //her køre den frem med speedselect fart
  motors.setSpeeds(speedselect, speedselect); //køre frem
}



void moveUntilLine(){
  drive0(50);               // her har jeg sat speedselect fart ind
  while(!sensorsState.L && !sensorsState.LC && !sensorsState.C && !sensorsState.RC && !sensorsState.R){
    readSensors(sensorsState); 
  }
 motors.setSpeeds(0,0);
 //LCD.clear();
 }


//void lcd(){
 // LCD.print("small");
 // LCD.gotoXY(0,1);
  
//}

void readSensors(LineSensorsWhite &State){
    // Next line reads the sensor values and store them in the array lineSensorValues 

    lineSensors.read(lineSensorValues, useEmitters ? QTR_EMITTERS_ON : QTR_EMITTERS_OFF); 

    // In the following lines use the values of the sensors to update the struct
    if (lineSensorValues[0] < threshold) sensorsState.L = 1;
    if (lineSensorValues[1] < threshold) sensorsState.LC = 1;
    if (lineSensorValues[2] < threshold) sensorsState.C = 1;
    if (lineSensorValues[3] < threshold) sensorsState.RC = 1;
    if (lineSensorValues[4] < threshold) sensorsState.R = 1;
}
void printSensorValues() {
  Serial.println("L: " + (String)lineSensorValues[0] + "\n" + "LC: " + (String)lineSensorValues[1] + "\n" + "C: " + (String)lineSensorValues[2] + "\n" + "RC: " + (String)lineSensorValues[3] + "\n" + "R: "  + (String)lineSensorValues[4] + "\n");
}
