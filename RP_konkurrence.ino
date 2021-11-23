#include<Wire.h>
#include<Zumo32U4.h> //Include the needed libraries
#define NUM_SENSORS 5


Zumo32U4Motors motors;
Zumo32U4Encoders encoders;    //initialize the different Zumo-functionalities
Zumo32U4ButtonA buttonA;
Zumo32U4LineSensors lineSensors;
Zumo32U4IMU imu;
Zumo32U4LCD lcd;
Zumo32U4ProximitySensors proxSensors;

/* turnAngle is a 32-bit unsigned integer representing the amount
the robot has turned since the last time turnSensorReset was
called.  This is computed solely using the Z axis of the gyro, so
it could be inaccurate if the robot is rotated about the X or Y
axes.

Our convention is that a value of 0x20000000 represents a 45
degree counter-clockwise rotation.  This means that a uint32_t
can represent any angle between 0 degrees and 360 degrees.  If
you cast it to a signed 32-bit integer by writing
(int32_t)turnAngle, that integer can represent any angle between
-180 degrees and 180 degrees. */
uint32_t turnAngle = 0;

// turnRate is the current angular rate of the gyro, in units of
// 0.07 degrees per second.
int16_t turnRate;

// This is the average reading obtained from the gyro's Z axis
// during calibration.
int16_t gyroOffset;

// This variable helps us keep track of how much time has passed
// between readings of the gyro.
uint16_t gyroLastUpdate = 0;

//Variable containing the distance between robot and center of line
int newDistanceFromCenter = 0;

//variable to store the data from the sensors
int lineSensorValues[NUM_SENSORS]; 


// White threshold; white returns values lower than this.
int threshold[NUM_SENSORS] = {300,200,160,200,230}; //{270,160,120,160,200};


// White threshold on belt
int thresholdBelt[NUM_SENSORS] = {0,0,0,0,0};

// Bool to turn_on/off the sensors
bool useEmitters = true;  


int canType; //Variable for determining can size. Value 0 is no can, 1 is small can, 2 is big can.

unsigned int brightnessLevels[6]; //Array which the setBrightnessLevels function retrieves its brightness levels from

//Variable for the robots stage
int stage = 1;

struct LineSensorsWhite { // True if White, False if Black
bool L; 
bool LC; 
bool C; 
bool RC; 
bool R;
};

LineSensorsWhite sensorsState = {0,0,0,0,0};


bool canRemoved = 0;

// Variables for position of the zumo
const int vector_max = 5;

double storage_vector[vector_max][2];

int angleFromStart = 0;



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);  // Start the serial-communication between the robot and the computer.
  lineSensors.initFiveSensors(); // Initialize the 5 lineSensors.
  proxSensors.initFrontSensor(); //Activates the front proximity sensor
  for(int i = 0; i<6; i++){ //This loop fills up our array with numbers from 1-7
    brightnessLevels[i] = i + 1;
  }
  proxSensors.setBrightnessLevels(brightnessLevels, 7); //This loop changes our brightness values to our custom ones, instead of the default ones. Default is too inaccurate for our usecase.
  calibrateThreshold();
  buttonA.waitForPress();
  buttonA.waitForRelease();
  turnSensorSetup();
  delay(1000);
  turnSensorReset();
}

void loop() {

  if (stage == 1) {
    displayStage();
    findLineAndIRSensor();
  }
  if (stage == 2) {
    displayStage();
    detectCan();
  }
  
  if (stage == 3) {
    displayStage();
    removeSmallCan();
    canRemoved = 0;
  }

  if (stage == 4) {
    displayStage();
    removeBigCan();
    canRemoved = 1;
  }

  if (stage == 5) {
    displayStage();
    hardCodeReturn();
  }
}

//
//
// Functions involving general movements of the robot
//
//

void displayStage() {
    lcd.clear();
    lcd.print("Stage: ");
    lcd.gotoXY(0,1);
    lcd.print(String(stage));
}

void calibrateThreshold() {
    //local variabel til sorte og hvide værdier
    int black[NUM_SENSORS] = {0,0,0,0,0};
    int white[NUM_SENSORS] = {0,0,0,0,0};
    int belt[NUM_SENSORS] = {0,0,0,0,0};
    
    //print "Placer over 'sort' og tryk på button A"
    lcd.clear();
    lcd.print("Place");
    lcd.gotoXY(0,1);
    lcd.print("black");
    
    // Vent på button A og aflæs sensorer
      buttonA.waitForPress();
      buttonA.waitForRelease();
      readSensors(sensorsState);
      printSensorValues();
      
    // Gemme de sorte værdier
     for (int i = 0; i < 5; i++) {
      black[i] = lineSensorValues[i];
     }
     
    //print "placer over hvid og tryk på button A"
    lcd.clear();
    lcd.print("Place");
    lcd.gotoXY(0,1);
    lcd.print("white");

    // vent på button a og aflæs sensorer
    buttonA.waitForPress();
    buttonA.waitForRelease();
    readSensors(sensorsState);
    printSensorValues();

    //lcd skal sige "placer ved start"
    lcd.clear();
    lcd.print("Place");
    lcd.gotoXY(0,1);
    lcd.print("belt");
    
    // gem de hvide værdier
    for (int i = 0; i < 5; i++) {
      white[i] = lineSensorValues[i];
     }

    //lcd skal sige "placer ved start"
    lcd.clear();
    lcd.print("Place");
    lcd.gotoXY(0,1);
    lcd.print("start");
    
    
    //læg hvid og sort sammen parvis, og divider med 2, brug derefter dette som threshold
    for (int i = 0; i < 5; i++) {
      threshold[i] = (black[i] + white[i])/2;
    }   
    Serial.println(String(threshold[0]) + ", " + String(threshold[1]) + ", " + String(threshold[2]) +", " + String(threshold[3]) + ", " + String(threshold[4]));

    //læg belt og hvid sammen parvis ....
    for (int i = 0; i < 5; i++) {
      thresholdBelt[i] = white[i] + 5;
    } 
   
}

void moveUntilLine() {
  moveForwardNoStop(100);
  sensorsState = {0,0,0,0,0};
  while(!sensorsState.L && !sensorsState.LC && !sensorsState.C &&!sensorsState.RC && !sensorsState.R) {
       readSensors(sensorsState);
  }
  stop();
}

void moveForward(int fart, double distance) {
  double globalMovement = 0; //initierer variabler med globalMovement og counts
  double counts = encoders.getCountsAndResetLeft(); //Resetter venstre encoder og sætter counts til det førhenværende antal counts
  counts = encoders.getCountsLeft(); // Henter den resettede encoder-data (Skulle gerne være 0)
  globalMovement = (counts/900) * PI * 3.9;
  motors.setSpeeds(fart,fart);
  while (globalMovement < distance) {
    counts = encoders.getCountsLeft();
    globalMovement = (counts/900) * PI * 3.9;
  }
  motors.setSpeeds(0,0);
  globalMovement = 0;
  counts = encoders.getCountsAndResetLeft();
}

void moveForwardNoStop(int fart) {
  motors.setSpeeds(fart,fart);
}

void stop() {
  motors.setSpeeds(0,0);
}

void turn(int fart, int grader, char direction) {
  if (direction == 'l' || direction == 'L') {
       turnSensorReset();
       while (((((int32_t)turnAngle >> 16) * 360) >> 16) < grader) {
         motors.setSpeeds(-fart,fart);
         turnSensorUpdate();
      }
      motors.setSpeeds(0,0);
  } else if (direction == 'r' || direction == 'R') {
       turnSensorReset();
       while (((((int32_t)turnAngle >> 16) * 360) >> 16) > -grader) {
          motors.setSpeeds(fart,-fart);
          turnSensorUpdate();
       }
       motors.setSpeeds(0,0);
  } 
}

//
//
// Other functions
//
//

void faceTowardIR() {
  moveForward(50,6);
  turn(100, 90,'r');
}

void readSensors(LineSensorsWhite &state){
    // Next line reads the sensor values and store them in the array lineSensorValues 

    lineSensors.read(lineSensorValues, useEmitters ? QTR_EMITTERS_ON : QTR_EMITTERS_OFF); 

    // In the following lines use the values of the sensors to update the struct
    if (stage < 3) {
      
    sensorsState.L = (lineSensorValues[0] < threshold[0]) ? 1 : 0;
    sensorsState.LC = (lineSensorValues[1] < threshold[1]) ? 1 : 0;
    sensorsState.C = (lineSensorValues[2] < threshold[2]) ? 1 : 0;
    sensorsState.RC = (lineSensorValues[3] < threshold[3]) ? 1 : 0;
    sensorsState.R = (lineSensorValues[4] < threshold[4]) ? 1 : 0;
    
    } else {
      
    sensorsState.L = (lineSensorValues[0] < thresholdBelt[0]) ? 1 : 0;
    sensorsState.LC = (lineSensorValues[1] < thresholdBelt[1]) ? 1 : 0;
    sensorsState.C = (lineSensorValues[2] < thresholdBelt[2]) ? 1 : 0;
    sensorsState.RC = (lineSensorValues[3] < thresholdBelt[3]) ? 1 : 0;
    sensorsState.R = (lineSensorValues[4] < thresholdBelt[4]) ? 1 : 0;
    }
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
        motors.setSpeeds(-70,100);
      }
      stop();
      break;

      case 1:
      while(!sensorsState.L) {
        readSensors(sensorsState);
        motors.setSpeeds(100,-70);
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


//
//
//Functions involving the gyro and linesensors
//
//

void turnSensorSetup()
{
  Wire.begin();
  imu.init();
  imu.enableDefault();
  imu.configureForTurnSensing();

  lcd.clear();
  lcd.print(F("Gyro cal"));

  // Turn on the yellow LED in case the LCD is not available.
  ledYellow(1);

  // Delay to give the user time to remove their finger.
  delay(500);

  // Calibrate the gyro.
  int32_t total = 0;
  for (uint16_t i = 0; i < 1024; i++)
  {
    // Wait for new data to be available, then read it.
    while(!imu.gyroDataReady()) {}
    imu.readGyro();

    // Add the Z axis reading to the total.
    total += imu.g.z;
  }
  ledYellow(0);
  gyroOffset = total / 1024;

  // Display the angle (in degrees from -180 to 180) until the
  // user presses A.
  lcd.clear();
  turnSensorReset();
  while (!buttonA.getSingleDebouncedRelease())
  {
    turnSensorUpdate();
    lcd.gotoXY(0, 0);
    lcd.print((((int32_t)turnAngle >> 16) * 360) >> 16);
    lcd.print(F("   "));
  }
  lcd.clear();
}

// This should be called to set the starting point for measuring
// a turn.  After calling this, turnAngle will be 0.
void turnSensorReset()
{
  gyroLastUpdate = micros();
  turnAngle = 0;
}

// Read the gyro and update the angle.  This should be called as
// frequently as possible while using the gyro to do turns.
void turnSensorUpdate()
{
  // Read the measurements from the gyro.
  imu.readGyro();
  turnRate = imu.g.z - gyroOffset;

  // Figure out how much time has passed since the last update (dt)
  uint16_t m = micros();
  uint16_t dt = m - gyroLastUpdate;
  gyroLastUpdate = m;

  // Multiply dt by turnRate in order to get an estimation of how
  // much the robot has turned since the last update.
  // (angular change = angular velocity * time)
  int32_t d = (int32_t)turnRate * dt;

  // The units of d are gyro digits times microseconds.  We need
  // to convert those to the units of turnAngle, where 2^29 units
  // represents 45 degrees.  The conversion from gyro digits to
  // degrees per second (dps) is determined by the sensitivity of
  // the gyro: 0.07 degrees per second per digit.
  //
  // (0.07 dps/digit) * (1/1000000 s/us) * (2^29/45 unit/degree)
  // = 14680064/17578125 unit/(digit*us)
  turnAngle += (int64_t)d * 14680064 / 17578125;
}



void followLine() {
  sensorsState.L = 0;
  sensorsState.R = 0;
  do {
    if (lineSensorValues[3] > threshold[3] * 1.1) {
      motors.setSpeeds(20,100);
    } else if (lineSensorValues[3] < threshold[3] * 0.9) {
      motors.setSpeeds(100,20);
    } else {
      motors.setSpeeds(60,60);
    }
    readSensors(sensorsState);
    printSensorValues();
  } while(!sensorsState.L || !sensorsState.R);
  alignAndCorrect();
}



//
//
// Proximity sensor functions
//
//

void detectCan(){
  while(true){
    proxSensors.read(); //Reads proximity sensors
    int proximityLeft = proxSensors.countsFrontWithLeftLeds();
    int proximityRight = proxSensors.countsFrontWithRightLeds(); //Retrieves the values from the read
    Serial.println("proxLeft: " + String(proximityLeft) + " // " + "proxRight: " + String(proximityRight));
    //Serial print for analysis
    if(proximityRight && proximityLeft >= 3){ //checks if it passes the threshold for the smallest can
      lineSensors.emittersOn();
      delay(400); //delay to let the can pass in front of the robot to avoid wrong decision from preliminary reading.
      lineSensors.emittersOff();
      proxSensors.read();
      int proximityLeft = proxSensors.countsFrontWithLeftLeds();
      int proximityRight = proxSensors.countsFrontWithRightLeds(); //reads sensors and updates the value, since the can is now more in front of the robot
      if(proximityRight && proximityLeft >= 7){ //Checks if it is a big can
        canType = 2; //Define can as type 2 = big can
        Serial.println("Last reading big: proxLeft: " + String(proximityLeft) + " // " + "proxRight: " + String(proximityRight)); //Check what value triggered the if statement. Used for debugging
        stage = 4;
        break; //Can is identified: exit the void
      }
      else
      canType = 1; //Since it wasn't a big can, define it as can 1 = small can
      Serial.println("Last reading small: proxLeft: " + String(proximityLeft) + " // " + "proxRight: " + String(proximityRight)); //Check what value triggered the if statement. Used for debugging
      stage = 3;
      break; //Can is identified: exit the void
    }
    lineSensors.emittersOn();
    delay(50);
    lineSensors.emittersOff();
  }
  //if none of the requirements are fulfilled, it loops back.
}


void removeBigCan() {
    //display that big can is found
    lcd.clear();
    lcd.print("Big can");
    
    // navigate behind big can
    turn(100, 90, 'r'); //Turn 90 degreees to the right
    moveForward(150, 27); //move forward for 35 cm
    turn(100, 90, 'l'); //Turn left 90 degrees times 2 (can't pick 180, cause then it just changes from 179 to -180 and not 180.
    moveForward(100, 20);
    turn(100, 90, 'l');
    delay(50);
    
    //Stop when line is detected then align with line
    moveUntilLine();
    alignAndCorrect();
    
    //Jump to stage 5
    stage = 5;
}

void removeSmallCan() {
  lcd.clear();
  delay(100);
  lcd.print("smallCan");
  delay(50);
  moveForward(100, 10.0);
  moveUntilLine();
  delay(50);
  stage = 5;
}

void findLineAndIRSensor() {
         //Read the data from the lineSensors
      readSensors(sensorsState);
   
       // if none of the sensors are white, moveforward until one of them is true
       moveUntilLine();
       
       // Correct the robot if it approaces at a angle that is not 90 deg
       alignAndCorrect(); // Assures that the Zumo robot is almost perpendicular to the white line.
       
      //Move the robot slightly forward so that it stands on top of the line and turn 90 degres
       faceTowardIR();

      // Move forward until left or right lineSensors detect white. 
       followLine();

      // Move forward to make sure IR sensor registers the shining light
        moveForward(100,0.5);

      //Move to next stage
        stage = 2;
 }


 void hardCodeReturn(){
    if(canRemoved == 0){
      turn(100, 135, 'r');
      moveForward(100, 30);
      turn(100, 45, 'r');
      moveForward(100, 10);
      turn(100, 90, 'r');
    }
    else if(canRemoved == 1){
      turn(100, 180, 'r');
      moveForward(100, 45);
      turn(100, 90, 'r');
      moveForward(100, 25);
      turn(100, 90, 'r');
    }
    else{
      lcd.clear();
      lcd.print("Error");
    }
    stage = 1;
  }
