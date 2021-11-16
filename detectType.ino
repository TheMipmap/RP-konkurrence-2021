//Import libraries and initialize proximity sensors.
#include <Zumo32U4.h>
#include <Wire.h>
Zumo32U4ProximitySensors proxSensors;
Zumo32U4LineSensors lineSensors;
int canType; //Variable for determining can size. Value 0 is no can, 1 is small can, 2 is big can.
unsigned int brightnessLevels[6]; //Array which the setBrightnessLevels function retrieves its brightness levels from

void detectCan(){
  while(true){
    proxSensors.read(); //Reads proximity sensors
    int proximityLeft = proxSensors.countsFrontWithLeftLeds();
    int proximityRight = proxSensors.countsFrontWithRightLeds(); //Retrieves the values from the read
    Serial.println("proxLeft: " + String(proximityLeft) + " // " + "proxRight: " + String(proximityRight));
    //Serial print for analysis
    if(proximityRight && proximityLeft >= 1){ //checks if it passes the threshold for the smallest can
      lineSensors.emittersOn();
      delay(400); //delay to let the can pass in front of the robot to avoid wrong decision from preliminary reading.
      lineSensors.emittersOff();
      proxSensors.read();
      int proximityLeft = proxSensors.countsFrontWithLeftLeds();
      int proximityRight = proxSensors.countsFrontWithRightLeds(); //reads sensors and updates the value, since the can is now more in front of the robot
      if(proximityRight && proximityLeft >= 5){ //Checks if it is a big can
        canType = 2; //Define can as type 2 = big can
        Serial.println("Last reading: proxLeft: " + String(proximityLeft) + " // " + "proxRight: " + String(proximityRight)); //Check what value triggered the if statement. Used for debugging
        break; //Can is identified: exit the void
      }
      else
      canType = 1; //Since it wasn't a big can, define it as can 1 = small can
      Serial.println("Last reading: proxLeft: " + String(proximityLeft) + " // " + "proxRight: " + String(proximityRight)); //Check what value triggered the if statement. Used for debugging
      break; //Can is identified: exit the void
    }
    lineSensors.emittersOn();
    delay(50);
    lineSensors.emittersOff();
  }
  //if none of the requirements are fulfilled, it loops back.
}

void setup() {
  Serial.begin(9600);
  lineSensors.initThreeSensors(); //Activates the three linesensors used for driving the conveyor belt
  proxSensors.initFrontSensor(); //Activates the front proximity sensor
  for(int i = 0; i<6; i++){ //This loop fills up our array with numbers from 1-7
    brightnessLevels[i] = i + 1;
  }
  proxSensors.setBrightnessLevels(brightnessLevels, 7); //This loop changes our brightness values to our custom ones, instead of the default ones. Default is too inaccurate for our usecase.
}

void loop() {
  detectCan();
  Serial.println("canType: " + String(canType));
  delay(4000);
}
