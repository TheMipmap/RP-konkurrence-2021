#include<Wire.h>
#include<Zumo32U4.h> //Include the needed libraries
#define NUM_SENSORS 5


Zumo32U4Motors motors;
Zumo32U4Encoders encoders;    //initialize the different Zumo-functionalities
Zumo32U4ButtonA buttonA;
Zumo32U4LineSensors lineSensors;
Zumo32U4IMU imu;
Zumo32U4LCD lcd;

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
int threshold[NUM_SENSORS] = {190,150,120,150,190}; 

// Bool to turn_on/off the sensors
bool useEmitters = true;  

//Variable for the robots stage
int stage = 0;

struct LineSensorsWhite { // True if White, False if Black
bool L; 
bool LC; 
bool C; 
bool RC; 
bool R;
};

LineSensorsWhite sensorsState = {0,0,0,0,0};


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);  // Start the serial-communication between the robot and the computer.
  lineSensors.initFiveSensors(); // Initialize the 5 lineSensors.
  turnSensorSetup();
  delay(500);
  turnSensorReset();
  buttonA.waitForPress(); //Starts the program after a click on button A and a small delay
  delay(100);
}

void loop() {

   if (stage == 0) {
       //Read the data from the lineSensors
      readSensors(sensorsState);
   
       // if none of the sensors are white, moveforward until one of them is true
       while(!sensorsState.L && !sensorsState.LC && !sensorsState.C &&!sensorsState.RC && !sensorsState.R) {
         readSensors(sensorsState);
          moveForwardNoStop(100);
       }
       stop();
   
       // Correct the robot if it approaces at a angle that is not 90 deg
       alignAndCorrect(); // Assures that the Zumo robot is almost perpendicular to the white line.
   
      //Move the robot slightly forward so that it stands on top of the line and turn 90 degres
       faceTowardIR();

      // Move forward until left and right lineSensors detect white. 
       followLine();
       stage = 1;
  }
}

//
//
// Functions involving general movements of the robot
//
//

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
  moveForward(50,4.4);
  turn(100, 90,'r');
}

void readSensors(LineSensorsWhite &state){
    // Next line reads the sensor values and store them in the array lineSensorValues 

    lineSensors.read(lineSensorValues, useEmitters ? QTR_EMITTERS_ON : QTR_EMITTERS_OFF); 

    // In the following lines use the values of the sensors to update the struct
    if (lineSensorValues[0] < threshold[0]) sensorsState.L = 1;
    if (lineSensorValues[1] < threshold[1]) sensorsState.LC = 1;
    if (lineSensorValues[2] < threshold[2]) sensorsState.LC = 1;
    if (lineSensorValues[3] < threshold[3]) sensorsState.RC = 1;
    if (lineSensorValues[4] < threshold[4]) sensorsState.R = 1;
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
        motors.setSpeeds(0,100);
      }
      stop();
      break;

      case 1:
      while(!sensorsState.L) {
        readSensors(sensorsState);
        motors.setSpeeds(100,0);
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

void calibrateLineSensors()
{
  lcd.clear();

  // Wait 1 second and then begin automatic sensor calibration
  // by rotating in place to sweep the sensors over the line
  delay(1000);
  for(uint16_t i = 0; i < 120; i++)
  {
    if (i > 30 && i <= 90)
    {
      motors.setSpeeds(-200, 200);
    }
    else
    {
      motors.setSpeeds(200, -200);
    }

    lineSensors.calibrate();
  }
  motors.setSpeeds(0, 0);
}

void followLine() {

  while(!sensorsState.L || !sensorsState.R) {
    
     //CalibrateLineSensors to the line
     calibrateLineSensors();

     //Get the position of the line
     int position = lineSensors.readLine(lineSensorValues);

     //As position 2000 correnspones to the center of the line we will have to use that as an indicator
     int distanceFromCenter = position - 2000;

     //
     int speedDifference = distanceFromCenter / 4 + 6 * (distanceFromCenter - newDistanceFromCenter);


     //make a new variable to correct from on the next iteration
     newDistanceFromCenter = distanceFromCenter;

     //Make the motors run at different speeds, depending on where the line is.
     int leftSpeed = 100 + speedDifference;
     int rightSpeed = 100 - speedDifference;


     //make sure none of the motors moves at a speed higher than 100 or lower than 0
     leftSpeed = constrain(leftSpeed, 0, 100);
     rightSpeed = constrain(rightSpeed, 0, 100);

     motors.setSpeeds(leftSpeed, rightSpeed);
  }
  stop();
  
}
