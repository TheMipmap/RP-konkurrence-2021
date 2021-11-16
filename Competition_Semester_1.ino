#include <Zumo32U4.h>
#include <Wire.h>

Zumo32U4Motors motors;
Zumo32U4IMU imu;
Zumo32U4ButtonA buttonA;
Zumo32U4LCD lcd;
Zumo32U4ButtonB buttonB;
Zumo32U4Encoders encoders;


int turnSpeed = 250;
int motorSpeed = 200;
const float Pi = 3.14159;
uint32_t turnAngle = 0;
int16_t turnRate;
int16_t gyroOffset;
uint16_t gyroLastUpdate = 0;


void setup() {
  // put your setup code here, to run once:

turnSensorSetup();
delay(500);
turnSensorReset();
lcd.clear();
}

// Code to use gyro, taken from Carlos :) 
void turnSensorReset()
{
  gyroLastUpdate = micros();
  turnAngle = 0;
}

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




//Move forward a certain distance(cm) using the encoders. 
//We also control the speed:
void moveForward (int speed, int centimeters) {
  encoders.getCountsAndResetLeft();
  motors.setSpeeds(motorSpeed,motorSpeed);
  double distance = 0;
  double counts_total = 0;
  do{
    delay(1);
    double counts = encoders.getCountsAndResetLeft();
    counts_total = counts_total + counts;
    distance = (counts_total  / 1204.44) * Pi * 3.9;
  } while
    (distance < centimeters);
    motors.setSpeeds(0,0);
}




//The ZUMO turns to the left, using degrees.
void turnLeft (int degrees) {
  turnSensorReset();
  motors.setSpeeds(0, turnSpeed);
  int angle = 0;
  do{
    delay(1);
    turnSensorUpdate();
    angle = (((int32_t)turnAngle >> 16)* 360)>> 16;
    lcd.gotoXY(0,0);
    lcd.print(angle);
    lcd.print(" ");
  } while (angle < degrees);
   motors.setSpeeds(0,0);
}





//The ZUMO turns to the right, using degrees.
void turnRight(int degrees) {
  turnSensorReset();
  motors.setSpeeds(turnSpeed,-turnSpeed);
  int angle = 0;
  do{
    delay(1);
    turnSensorUpdate();
    angle = (((int32_t)turnAngle >> 16)*360)>> 16;
    lcd.gotoXY(0,0);
    lcd.print(angle);
    lcd.print(" ");
  } while (angle > -degrees);
  motors.setSpeeds(0,0);
    
  }

void loop() {
  // put your main code here, to run repeatedly:

turnSensorUpdate();

//We can delete this when merging:
  buttonB.waitForButton();
  delay(500);
//Delete part for competition ^^^

//The real deal:
  turnRight(90); //Turn 90 degreees to left
  moveForward(motorSpeed,35); //move forward for 35 cm
  delay(50);
  turnLeft(179); //Turn left 179 degrees (can't pick 180, cause then it just changes from 179 to -180 and not 180.
  
  //Replace this part with "Stop at white line"
  motors.setSpeeds(motorSpeed,motorSpeed);
  //Delete it for competition ^^^
 }
