#include<Wire.h>
#include<Zumo32U4.h>

Zumo32U4Motors motors;
Zumo32U4Encoders encoders;
Zumo32U4ButtonA buttonA;
Zumo32U4IMU imu;
Zumo32U4LCD lcd;

uint32_t turnAngle = 0;
int angleFromStart = 0;

// turnRate is the current angular rate of the gyro, in units of
// 0.07 degrees per second.
int16_t turnRate;

// This is the average reading obtained from the gyro's Z axis
// during calibration.
int16_t gyroOffset;

// This variable helps us keep track of how much time has passed
// between readings of the gyro.
uint16_t gyroLastUpdate = 0;

const int vector_max = 5;
double storage_vector[vector_max][2];




void setup() {
  Serial.begin(9600);  // Start the serial-communication between the robot and the computer.
  turnSensorSetup();
  delay(500);
  turnSensorReset();
  buttonA.waitForPress(); //Starts the program after a click on button A and a small delay
  delay(1000);
}

void loop() {
  moveForward(200, 20);
  vectorize(0);
  turn(100, 90, 'r');
  moveForward(200, 20);
  vectorize(1);
  turn(100, 30, 'l');
  moveForward(200, 15);
  vectorize(2);
  moveForward(200, 15);
  vectorize(3);
  turn(100, 99, 'r');
  moveForward(200, 18);
  vectorize(4);
  returnHome();
  buttonA.waitForPress();
}

void moveForward(int speed, double distance) {
  double globalMovement = 0; //initierer variabler med globalMovement og counts
  double counts = encoders.getCountsAndResetLeft(); //Resetter venstre encoder og sætter counts til det førhenværende antal counts
  counts = encoders.getCountsLeft(); // Henter den resettede encoder-data (Skulle gerne være 0)
  globalMovement = (counts / 900) * PI * 3.9;
  motors.setSpeeds(speed, speed);
  while (globalMovement < distance) {
    counts = encoders.getCountsLeft();
    globalMovement = (counts / 900) * PI * 3.9;
  }
  motors.setSpeeds(0, 0);
  globalMovement = 0;

}

void turn(int speed, int grader, char direction) {
  if (direction == 'l' || direction == 'L') {
    turnSensorReset();
    angleFromStart += grader;
    while (((((int32_t)turnAngle >> 16) * 360) >> 16) < grader) {
      motors.setSpeeds(-speed, speed);
      turnSensorUpdate();
    }
    motors.setSpeeds(0, 0);
  } else if (direction == 'r' || direction == 'R') {
    turnSensorReset();
    angleFromStart -= grader;
    while (((((int32_t)turnAngle >> 16) * 360) >> 16) > -grader) {
      motors.setSpeeds(speed, -speed);
      turnSensorUpdate();
    }
    motors.setSpeeds(0, 0);

  }
}



  void turnSensorSetup() {
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
      while (!imu.gyroDataReady()) {}
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

  void turnSensorReset() {
    gyroLastUpdate = micros();
    turnAngle = 0;
  }

  // Read the gyro and update the angle.  This should be called as
  // frequently as possible while using the gyro to do turns.
  void turnSensorUpdate() {
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

  void vectorize(int i) {
    double counts = encoders.getCountsLeft();
    double rad = ((((int32_t)turnAngle >> 16) * 360) >> 16) * PI / 180;
    double x = counts * cos(rad);
    double y = counts * sin(rad);
    storage_vector[i][0] = x;
    storage_vector[i][1] = y;
    lcd.clear();
    lcd.print(storage_vector[i][0]);
    lcd.gotoXY(0, 1);
    lcd.print(storage_vector[i][1]);
    delay(1000);
  }

  void radian() {
    double rad = ((((int32_t)turnAngle >> 16) * 360) >> 16) * PI / 180;
    lcd.clear();
    lcd.print((String)rad);
    lcd.gotoXY(0, 1);
    lcd.print((String)angleFromStart);
    delay(1000);
  }

  void returnHome() {
    //sumvector is created
    double sumVector[2];
    double x_total = 0;
    double y_total = 0;

    //sumvector gets calculated
    for (int i = 0; i < vector_max; i++) {
      x_total += storage_vector[i][0];
    }
    for (int j = 0; j < vector_max; j++) {
      y_total += storage_vector[j][1];
    }
    sumVector[0] = (x_total / 900) * PI * 3.9;
    sumVector[1] = (y_total / 900) * PI * 3.9;

    //distance it needs to travel
    double returnDistance = sqrt(pow(sumVector[0], 2) + pow(sumVector[1], 2));

    //angel from sumvector
    double angleHome = 57.2957795 * atan2(sumVector[0], -sumVector[1]);

    lcd.clear();
    lcd.print("Angles");
    lcd.gotoXY(0, 1);
    lcd.print(angleHome);

    if (angleFromStart > 0) turn(100, 180 - angleFromStart, 'l');
    else if (angleFromStart <= 0) turn(100, 180 + angleFromStart, 'r');

    turn(100, angleHome, 'r');
    moveForward(200, returnDistance);
    turn(100,90+angleHome,'r');





  }
