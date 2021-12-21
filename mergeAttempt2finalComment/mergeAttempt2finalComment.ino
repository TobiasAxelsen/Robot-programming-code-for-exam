//Included libraries
#include <Wire.h>
#include <Zumo32U4.h>
#include "TurnSensor.h"
#include<math.h>
Zumo32U4LineSensors lineSensors;
Zumo32U4ButtonA buttonA;
Zumo32U4Motors motors;
Zumo32U4LCD lcd;
Zumo32U4Encoders encoders;
Zumo32U4IMU imu;
Zumo32U4Buzzer buzzer;
Zumo32U4ProximitySensors proxSensors;

// variables used for proximity
const int arraySize = 30; // this value determines the resolution of the data received from the proximity sensor 
unsigned int brightnessLevels[arraySize];
unsigned int proxReading[2];
unsigned int canType;

// these two arrays store the calibrated linesensor values 
int blackValues[5] = {};
int whiteValues[5] = {};

#define NumSensors 5
unsigned int lineSensorValues[NumSensors];
int16_t lastError = 0;
const uint16_t maxSpeeds = 200;

int sensorThreshold = 700;
int globalStage = 0;
bool runOnce_findLineAndDrive = false;
bool lineFound = false;
bool useEmitters = true;
double accCountsR = 0;
int stage = 0;
bool buttonState;

int thresholds[5] = {800, 300, 230, 300, 800};

// --------------setup----------------//

void setup() {
  delay(3000);
  Serial.begin(9600);
  findLineAndDriveSetup();
  initiateDetection();
}


void loop() {
  //following switch loop is used to control the flow of the program using a global variable called globalStage
  //as this variable changes wwe progress through the loop and run the different functions
  switch (globalStage) {
    case 0:
      findLineAndDrive();
      break;

    case 1:
      lcd.clear();
      lcd.print("case 1");
      canDetection();
      break;

    case 2:
      lcd.clear();
      lcd.print("case 2");
      smallCan();
      break;

    case 3:
      lcd.clear();
      lcd.print("case 3");
      bigCan();
      break;

    default:
      Serial.println("case error");
  }

}


//-------------------- findlineanddrivesetup ---------------------//

//this function is used to setup the gyroscope as well as the IR-LineSensor
void findLineAndDriveSetup() {
  lcd.clear();
  lcd.print("Press A");
  lcd.gotoXY(0, 1);
  lcd.print("to calib");
  buttonA.waitForButton();
  lcd.clear();

  turnSensorSetup();
  delay(500);
  turnSensorReset();
  delay(500);
  lineSensors.initFiveSensors();

  lcd.clear();
  lcd.print("IFsensor");
  lcd.gotoXY(0, 1);
  lcd.print("calib");
  delay(500);
  calibrateSensors();
  lcd.clear();


  lcd.print("Calib");
  lcd.gotoXY(0, 1);
  lcd.print("success!");

}


// -------------------------------  findlineanddrive ----------------------------- //

//following function is used to find the line and follow it, some of this code is most likely redundant however due to time constraints
//it has been decided to keep it in as the program currently is in a fully functioning state
void findLineAndDrive() {
  if (runOnce_findLineAndDrive == false) {
    lcd.clear();
    lcd.print("Press A");
    lcd.gotoXY(0, 1);
    lcd.print("to drive");
    buttonA.waitForButton();
    lcd.clear();
    runOnce_findLineAndDrive = true;
  }


  //Computing linefollowing speeds on each motor
  int16_t position = lineSensors.readLine(lineSensorValues);
  int16_t error = position - 2000;
  int16_t speedDifference = error / 4 + 6 * (error - lastError);
  lastError = error;
  int16_t leftSpeed = (int16_t)maxSpeeds + speedDifference;
  int16_t rightSpeed = (int16_t)maxSpeeds - speedDifference;


  // this switch is used to control the flow of the function
  switch (stage) {
    case 0: //Finding the line
      //Serial.println(stage);
      //lineFound = false;
      if (lineFound == false) lineFound = findLine(200);
      else if (lineFound == true) stage++;
      break;

    case 1: //Placing the middle of the zumo on the line
      Serial.println(stage);
      alignAndCorrect();
      // we drive 6cm as we have measured that its almost equivalent to the length of the zumo
      driveDistance(6, 100);
      stage++;
      lineFound = false;
      break;

    case 2: //Taking a break
      Serial.println(stage);
      motorStop();



      stage++;
      break;

    case 3: //Turning 90 degrees right to line up with line
      turnSensorReset();
      turnSensorUpdate(); // Updates the turnsensor
      turnRight(90);
      stage++;
      break;

    case 4: //Following line
      lineFollowing();
      lcd.clear();
      lcd.print("Follow");
      lcd.gotoXY(0, 1);
      lcd.print("line");

      if (lineSensorValues [0] < 1000 && lineSensorValues[4] < 1000) { //This line changes whether or not it goes to next stage depending on white/black line
        lcd.clear();
        stage++;
      }
      break;

    case 5: //Parks the zumo
      motors.setSpeeds(0, 0);

      lcd.clear();
      lcd.print("Standing");
      lcd.gotoXY(0, 1);
      lcd.print("by...");

      delay(1000);

      globalStage = 1;

      break;
    default:
      Serial.println("stage switch case error");


  }
}

// -------------------------------- findLine --------------------------------- //

//the following function is used for finding the line either at the very start of the program or when we have returned from pushing a can
int findLine(int mSpeed) {

  bool thresholdExceeded = false;

  for (int i = 0; i < NumSensors; i++) {
    lineSensors.read(lineSensorValues, useEmitters ? QTR_EMITTERS_ON : QTR_EMITTERS_OFF);
    Serial.println(lineSensorValues[4]);
    if (sensorThreshold > lineSensorValues[4] || sensorThreshold > lineSensorValues[0]) thresholdExceeded = true; //-----------------This line also decides whether or not we follow a black or white line

    switch (thresholdExceeded) {
      case 0: //false
        motors.setSpeeds(mSpeed, mSpeed);
        break;

      case 1: //true
        motors.setSpeeds(0, 0);
        break;

      default:
        Serial.println("switch case findLine() error");
    }
    return thresholdExceeded;
  }

}


// ---------------------- line following -------------------//

//the line following code checks  if the middle sensor is on the line, if it begins to deviate from the line
//one of the sensors next to it will register and the motor values will be changed to adjust

void lineFollowing() {

  lineSensors.read(lineSensorValues, useEmitters ? QTR_EMITTERS_ON : QTR_EMITTERS_OFF);

  //if the robot begins to deviate to the right: make the right wheel spin faster
  if (lineSensorValues[2] < whiteValues[2] * 1.32) {

    motors.setSpeeds(100, 200);

    // if the robot begins to deviate to the left make the left wheel spin faster
  } else if (lineSensorValues[3] > whiteValues[3] * 0.80) {
    motors.setSpeeds(200, 100);
  } else { // if the robot is on the line both motors provide equally
    motors.setSpeeds(400, 400);
  }

}


// ------------------ calibrate Sensors -----------------//
//this function is used to calibrate the sensors and provide values to the global arrays for white and black threshold values
void calibrateSensors() {

  // following integer determines how many readings we take
  int calibrationAmount = 100;

  // these two arrays are used to calculate the sum of all the raeadings from the sensor
  int64_t blackCalSums[5] = {};
  int64_t whiteCalSums[5] = {};
  lcd.clear();
  lcd.print("black");
  lcd.gotoXY(0, 1);
  lcd.print("calibrat");



  buttonA.waitForPress();
  buttonA.waitForRelease();

  // as long as i is lower than our calibrationamount we take readings
  for (int i = 0; i < calibrationAmount; i++) {
    lineSensors.read(lineSensorValues, useEmitters ? QTR_EMITTERS_ON : QTR_EMITTERS_OFF);

    // we take the sum of the readings from each sensor and store them in a array
    for (int p = 0; p < 5; p++) {
      blackCalSums[p] += lineSensorValues[p];


    }
    // then we find the average and store them in our global array
    for ( int p = 0; p < 5; p++) {
      blackValues[p] = round((blackCalSums[p] / calibrationAmount));
    }
  }
  Serial.println((String)blackValues[0] + " " + (String)blackValues[1] + " " + (String)blackValues[2] + " " + (String)blackValues[3] + " " + (String)blackValues[4] + " ");


  lcd.clear();
  lcd.print("white");
  lcd.gotoXY(0, 1);
  lcd.print("calibrate");



  buttonA.waitForPress();
  buttonA.waitForRelease();

  // following for loops does the exact same but for the white values
  for (int i = 0; i < calibrationAmount; i++) {
    lineSensors.read(lineSensorValues, useEmitters ? QTR_EMITTERS_ON : QTR_EMITTERS_OFF);
    Serial.println(lineSensorValues[0]);
    for (int p = 0; p < 5; p++) {
      whiteCalSums[p] += lineSensorValues[p];


    }
    for (int p = 0; p < 5; p++) {
      whiteValues[p] = round((whiteCalSums[p] / calibrationAmount));
    }
  }

  Serial.println((String)whiteValues[0] + " " + (String)whiteValues[1] + " " + (String)whiteValues[2] + " " + (String)whiteValues[3] + " " + (String)whiteValues[4] + " ");

}
  //----------- turnRight ---------------------//


  void turnRight(int degrees) {
    turnSensorReset();
    motors.setSpeeds(200, -200);
    int angle = 0;
    lcd.gotoXY(0, 0);
    lcd.print("Turning");
    lcd.gotoXY(0, 0);
    lcd.print(" Right ");
    do {
      delay(1);
      turnSensorUpdate();

      // we calculate the angle in degrees
      angle = (((int32_t)turnAngle >> 16) * 360) >> 16;
    } while (angle > -degrees);
    motors.setSpeeds(0, 0);
    lcd.clear();
  }


  // ------------------- turnleft--------------//
  void turnLeft(int degrees) {
    turnSensorReset();
    turnSensorUpdate();
    motors.setSpeeds(-200, 200);
    int angle = 0;
    lcd.gotoXY(0, 0);
    lcd.print("Turning");
    lcd.gotoXY(0, 0);
    lcd.print(" Left ");
    do {
      delay(1);
      turnSensorUpdate();
      angle = (((int32_t)turnAngle >> 16) * 360) >> 16;
    } while (angle < degrees);
    motors.setSpeeds(0, 0);
    lcd.clear();
  }





  void motorStop() {
    motors.setSpeeds(0, 0);
  }


  //------------------------------- canDetection ---------------------//
  // this function uses the proximity sensor to check for a can in front of it
  void canDetection() {

    proxSensors.read();
    proxReading[0] = proxSensors.countsFrontWithLeftLeds();
    proxReading[1] = proxSensors.countsFrontWithRightLeds();
    Serial.println(String(proxReading[0]) + String(proxReading[1]));
    if ((proxReading[0] && proxReading[1] > 28) && (proxReading[0] && proxReading[1] < 30)) {
      // small can
      globalStage = 2;
    } else if (proxReading[0] && proxReading[1] >= 30) {
      // large can
      globalStage = 3;
    }
    else {
      // no can
      globalStage = 1;
    }
    delay(50);

    lineSensors.emittersOn();
    delay(50);
    lineSensors.emittersOff();

  }



  // -------------- initiate detection -----------------//
  //initiates the proximity sensors
  void initiateDetection() {
    proxSensors.initFrontSensor();
    delay(1000);
    for (int i = 0; i < arraySize; i++) {
      brightnessLevels[i] =  i + 1;
      Serial.println(String(brightnessLevels[i]));
    }
    proxSensors.setBrightnessLevels(brightnessLevels, arraySize + 1);
  }


  //------------------------------------smallCan-------------------//

  void smallCan() {

    int smallspeed = 200;
    int smallerspeed = 100;

    int blackThreshold = 500;
    lineSensors.read(lineSensorValues, useEmitters ? QTR_EMITTERS_ON : QTR_EMITTERS_OFF);

    // the zumo drives unto the belt without detection as the space between platform and belt can trigger the line sensor
    driveDistance(10, 200);

    // drive until the zumo sees the white line
    do  {
      motors.setSpeeds(smallspeed, smallspeed);
      lineSensors.read(lineSensorValues, useEmitters ? QTR_EMITTERS_ON : QTR_EMITTERS_OFF);
    } while ( lineSensorValues[4] > 700 );
    motors.setSpeeds(0, 0);
    alignAndCorrect();


    driveDistance(8, -200);
    turnRight(150);

    delay(200);
    driveDistance(25, 300);
    turnRight(100);


    stage = 0;
    globalStage = 0;

  }



  //----------------------- drive distance ---------------------//

  //Following function drives the zumo a set-distance
  void driveDistance(double lengthD, int dSpeed) {
    resetEncoders();
    readEncoders();
    double distance;
    distance = ((accCountsR / 900) * PI * 3.65);

    //while the absolute value of the distance is under the desired length the zumo drives
    // the reason we use the absolute value is so that we also can use this function to drive backwards by giving it a negative motorspeed value
    while (abs(distance) < lengthD) {
      readEncoders();
      distance = ((accCountsR / 900) * PI * 3.65);
      Serial.println("the distance currently is " + (String)distance);
      motors.setSpeeds(dSpeed, dSpeed);

    }

    motors.setSpeeds(0, 0);

    Serial.println("the final distance is " + (String)distance);
    Serial.println("the final accCountsR is " + (String)accCountsR);
    accCountsR = 0;
    distance = 0;
  }


  //------------------readencoders---------------------//

  void readEncoders() { //Reads encoders
    accCountsR = accCountsR + encoders.getCountsAndResetRight();
  }



  //----------------------- big can ---------------------//
  //function used when we detect a big can on the belt
  void bigCan() {

    int smallspeed = 200;
    int smallerspeed = 100;

    int blackThreshold = 500;
    resetEncoders();

    // as to ensure we enter the course from the right side we use our drive distance and turn functions to drive unto the belt
    turnRight(90);
    driveDistance(25, 200);
    turnLeft(90);
    resetEncoders(); // remember to insert the resetencoders into the drive distance function
    driveDistance(20, 200);
    turnLeft(80);

    // once the zumo is positioned correctly it drives until it detects the line at the end of the belt
    do  {
      motors.setSpeeds(smallspeed, smallspeed);
      lineSensors.read(lineSensorValues, useEmitters ? QTR_EMITTERS_ON : QTR_EMITTERS_OFF);
    } while ( lineSensorValues[2] > blackThreshold);
    motorStop();

    // once the white line has been reached the zumo returns
    driveDistance(25, -200);

    turnLeft(135);
    driveDistance(30, 400);

    turnRight(135);



    stage = 0;
    globalStage = 0;
  }

  //-----------------lcdDebugger -----------------------//
  //this function has been used to check if the program would get stuck at places in the code and write it to the lcd-screen

  void lcdDebugger() {
    lcd.clear();
    lcd.print("debugger");
    lcd.gotoXY(0, 1);
    lcd.print("reached");
    delay(2000);
  }

  void resetEncoders() {
    readEncoders();
    encoders.getCountsAndResetRight;
    encoders.getCountsAndResetLeft;
    accCountsR = 0;

    Serial.println("resetting the encoders to " + (String)accCountsR);
  }


  //------------- Align and correct ----------------------//

  // following function is used to make sure that the zumo is close to orthogonal on a line when it sees it
  // we accomplish this by looking at what of the outer sensors is first to detect the line and then rotate the zumo until the other also detcts it.
  void alignAndCorrect() {

    lcd.clear();
    lcd.print("aligning");
    lcd.gotoXY(0, 1);
    lcd.print("chassis");

    Serial.println("alignAndCorrect");
    int blackThreshold = 500;
    //Find out which lineSensors that turned white


    int sensorIsWhite;
    lineSensors.read(lineSensorValues, useEmitters ? QTR_EMITTERS_ON : QTR_EMITTERS_OFF);

    if (lineSensorValues[0] < thresholds[0]) sensorIsWhite = 0;
    if (lineSensorValues[1] < thresholds[1]) sensorIsWhite = 2;
    if (lineSensorValues[2] < thresholds[2]) sensorIsWhite = 2;
    if (lineSensorValues[3] < thresholds[3]) sensorIsWhite = 2;
    if (lineSensorValues[4] < thresholds[4]) sensorIsWhite = 1;


    switch (sensorIsWhite) { // this switch checks what sensor was the first to detect the line

      case 0: // if the outer left sensor is the first to detect the line we adjust the right motorspeed
        while (lineSensorValues[4] > 800) {
          lineSensors.read(lineSensorValues, useEmitters ? QTR_EMITTERS_ON : QTR_EMITTERS_OFF);

          motors.setSpeeds(0, 70);
        }
        motorStop();
        break;

      case 1:// if the outer right sensor is the first to detect the line we adjust the left motorspeed
        while (lineSensorValues[0] > 800) {
          lineSensors.read(lineSensorValues, useEmitters ? QTR_EMITTERS_ON : QTR_EMITTERS_OFF);
          motors.setSpeeds(70, 0);
        }
        motorStop();
        break;

      default:
        motorStop();
    }

  }
