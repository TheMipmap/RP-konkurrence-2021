#include<Wire.h>
#include<Zumo32U4.h> //Include the needed libraries
#define NUM_SENSORS 5


Zumo32U4Motors motors;
Zumo32U4Encoders encoders;    //initialize the different Zumo-functionalities
Zumo32U4ButtonA buttonA;
Zumo32U4LineSensors lineSensors;


int lineSensorValues[NUM_SENSORS]; //variable to store the data from the sensors
int threshold = 200; // White threshold; white returns values lower than this.

bool useEmitters = true;  // Bool to turn_on/off the sensors


struct LineSensorsWhite { // True if White, False if Black
bool L; 
bool LC; 
bool C; 
bool RC; 
bool R;
};

LineSensorsWhite sensorsState = {0,0,0,0,0};

LineSensorsWhite LSW;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);  // Start the serial-communication between the robot and the computer.
  lineSensors.initFiveSensors(); // Initialize the 5 lineSensors.
  buttonA.waitForPress(); //Starts the program after a click on button A and a small delay
  delay(100);
}

void loop() {
    //Read the data from the lineSensors
   readSensors(sensorsState);
   // if none of the sensors are white, moveforward until one of them is true
   while(!sensorsState.L && !sensorsState.LC && !sensorsState.C &&!sensorsState.RC && !sensorsState.R) {
      readSensors(sensorsState);
      moveForward(100);
   }
   stop();
   alignAndCorrect();
}


void moveForward(int fart) {
  motors.setSpeeds(fart,fart);
}

void stop() {
  motors.setSpeeds(0,0);
}

void readSensors(LineSensorsWhite &state){
    // Next line reads the sensor values and store them in the array lineSensorValues 

    lineSensors.read(lineSensorValues, useEmitters ? QTR_EMITTERS_ON : QTR_EMITTERS_OFF); 

    // In the following lines use the values of the sensors to update the struct
    if (lineSensorValues[0] < threshold) sensorsState.L = 1;
    if (lineSensorValues[1] < threshold) sensorsState.LC = 1;
    if (lineSensorValues[2] < threshold) sensorsState.LC = 1;
    if (lineSensorValues[3] < threshold) sensorsState.RC = 1;
    if (lineSensorValues[4] < threshold) sensorsState.R = 1;
}

void alignAndCorrect() {
    Serial.println("alignAndCorrect");
    //Find out which lineSensors that turned white
    readSensors(sensorsState);
    int sensorIsWhite;
    
      if (sensorsState.L) sensorIsWhite = 0;
      if (sensorsState.LC) sensorIsWhite = 2;
      if (sensorsState.C) sensorIsWhite = 2;
      if (sensorsState.RC) sensorIsWhite = 2;
      if (sensorsState.R) sensorIsWhite = 1;

      
   switch (sensorIsWhite) {
      case 0:
      while(!sensorsState.R) {
        readSensors(sensorsState);
        motors.setSpeeds(0,50);
      }
      stop();
      break;

      case 1:
      while(!sensorsState.L) {
        readSensors(sensorsState);
        motors.setSpeeds(50,0);
      }
      stop();
      break;

      default:
      stop();
   }
}


void printSensorValues() {
  Serial.println("L: " + (String)lineSensorValues[0] + "\n" + "LC: " + (String)lineSensorValues[1] + "\n" + "C: " + (String)lineSensorValues[2] + "\n" + "RC: " + (String)lineSensorValues[3] + "\n" + "R: "  + (String)lineSensorValues[4] + "\n");
}
