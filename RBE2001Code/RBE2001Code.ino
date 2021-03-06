/*
  TEAM 8 2001 Robot Code
  Main state machine
  By Danny Lu, Nicoli Liedtke, and David Jin
*/

#include "Arduino.h"
#include "Messages.h"
#include <Servo.h>
#include <PID_v1.h> //for Arm


Servo rightmotor; //Initalize actuators
Servo leftmotor; 
Servo arm;
Servo gripper;

Messages msg;
unsigned long timeForHeartbeat; //heartbeat count
int initializeMem = 0; //get bluetooth info
int dataCounter = 0;
bool lowRad = false; //Rad = Radiation
bool highRad = false;
bool radRodFull; //true means new rod, false means spent rod
bool sendRad = false; //Radiation signals
bool stopRobot = true;

bool dump1 = false; //memory for fuel rod positions
bool dump2 = false;
bool dump3 = false;
bool dump4 = false;
bool supply1 = true;
bool supply2 = true;
bool supply3 = true;
bool supply4 = true;

//Movement control booleans
bool up = false;
bool backOnWhite = true; //stops counting lines when on black
bool turnedAround = false;
bool grabbed = false;
bool intersectionFound = false;
bool turned90 = false;
bool firstReactor = true;

//Underglow LEDS
const int ledFlashLength = 120;
const int ledLength = 10;
int ledTime;
int brightness = 255;
int ledBrightness = 0;

int dumpToSupplyTurn; //0 is left, 1 is no turn, 2 is right turn
int dumpToSupplyLineSkips;
//V1
int absDumpToSupplyLineSkips;
int n;
int desiredDump;
int desiredSupply;

//Sensors and Limit values
const int lineCounter = A0;
const int lineLeft = A1;
const int lineRight = A2;
const int lineBackL = A3;
const int lineBackR = A5;
const int pot = A4;
const int estop = 23;
const int LEDS = 13;
const int bumpSwitch = 29;
const int black = 500;
const int armBack = 38;
const int armDown = 395;
const int armInsert = 115;
const int armInsertReactor = 340;
const int armPickupVal = 70;
int Kp = 3;
int Ki = 0;
int Kd = 0;
long desiredTime;
double setpoint, inputValue, outputValue;
PID pid(&inputValue, &outputValue, &setpoint, Kp, Ki, Kd, DIRECT);

//Main State Machine
static enum runStates {decideNextState, determineBluetooth, navigateReactor, navigateDump, navigateNewRod, navigateSupplyToReactor, pickUp, putDown, returnRod, getNewRod, finished}
runState;
//Lower Level State Machines
static enum turnAroundStates {setDesiredTime, backUp, skipLine, findLine}
turnAroundState;
static enum pickupStates {armDownRun, grip, armUp, turnAroundRun}
pickupState;
static enum navigateDumpStates {findIntersection, turnToDump, approachDump}
navigateDumpState;
static enum returnRodStates {armInsertRun, releaseGrip, armBackRun, turnAroundRunDump}
returnRodState;
static enum navigateNewRodStates {navigateCenterLine, turnOnCenterLine, moveToSupplyRow, turnToSupply, approachSupply}
navigateNewRodState;
static enum getNewRodStates {armRetrieveRun, closeGrip, armReverseRun, turnAroundRunSupply}
getNewRodState;
static enum navigateSupplyToReactorStates {goToCenterLine, turnToReactor, moveToReactor}
navigateSupplyToReactorState;
static enum putDownStates {wait, correctDistance, moveArmDown, releaseGripper, moveArmUp, turnAroundAfterDrop}
putDownState;


void setup() {
  // put your setup code here, to run once:
  pinMode(estop, INPUT_PULLUP);
  pinMode(LEDS, OUTPUT);
  pinMode(bumpSwitch, INPUT_PULLUP);
  leftmotor.attach(11, 1000, 2000); // left drive motor pin#, pulse time for 0,pulse time for 180
  rightmotor.attach(10, 1000, 2000); // right drive motor pin#, pulse time for 0,pulse time for 180
  arm.attach(9, 1000, 2000);
  gripper.attach(4, 1000, 2000);
  Serial.begin(115200);
  Serial.println("Starting");
  msg.setup();
  timeForHeartbeat = millis() + 1000;
  
  //Initialize all states
  runState = determineBluetooth;
  pickupState = armDownRun;
  turnAroundState = setDesiredTime;
  navigateDumpState = findIntersection;
  returnRodState = armInsertRun;
  navigateNewRodState = navigateCenterLine;
  getNewRodState = armRetrieveRun;
  navigateSupplyToReactorState = goToCenterLine;
  putDownState = wait;

  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-90, 90);
  bluetoothFunction();
  analogWrite(LEDS, 0);
}

void runRobot() {
  switch (runState) { //Main state machine
    case decideNextState:

      break;
    case determineBluetooth:
      bluetoothSetBooleans();
      setDesiredDumpAndSupply();
      determineTurnsAndLineSkips();


      break;
    case navigateReactor:

      lineFollow();
      if (digitalRead(bumpSwitch) == LOW) {
        stopMotors();
        runState = pickUp;
      }
      break;
    case navigateDump:
      navigateDumpFunction();
      break;
    case navigateNewRod:
      navigateNewRodFunction();
      break;
    case navigateSupplyToReactor:
      navigateSupplyToReactorFunction();
      break;
    case pickUp:
      pickupFunction();
      break;
    case putDown:
      putDownFunction();
      break;
    case returnRod:

      returnRodFunction();

      break;
    case getNewRod:
      getNewRodFunction();
      break;
    case finished:

      break;
  }

}
void loop() {
  if(digitalRead(estop)==LOW){ //e-stop reader
    msg.setStopped(true);
    Serial.print("AHHHHHH");
  }

  if (!stopRobot) { //for bluetooth stop and resume
    runRobot();
    if(sendRad){
      if(radRodFull){
        flash();
      }
      else{
        pulse();
      }
    }
    else{
      brightness = 70;
    }
  }
  else {
    stopMotors();
    arm.write(90);
    gripper.write(90);
    brightness = 0;
  }
  
  bluetoothFunction();
}

void bluetoothFunction() { //Bluetooth initalization and implementation
  if (msg.read()) {
    msg.updateFieldMem();
    if (initializeMem < 3) {
      initializeMem++;
    }
    else {
      //msg.printMessage();
      stopRobot = msg.isStopped();
    }
  }
  if (millis() > timeForHeartbeat) {
    timeForHeartbeat = millis() + 1000;
    msg.sendHeartbeat();
    if (dataCounter >= 4) {
      dataCounter = 0;
      if (sendRad) {
        msg.sendRadiation(radRodFull);
      }

      //msg.sendRobotStatus(0x02,0x01,0x04);
    }
    else {
      dataCounter++;
    }
  }
}

void lineFollow() { //Basic line following function
  if ((analogRead(lineLeft) > black) && (analogRead(lineRight) < black)) {
    driveLeft();
  }
  else if ((analogRead(lineRight) > black) && (analogRead(lineLeft) < black)) {
    driveRight();
  }
  else {
    driveStraight();
  }
}

void driveStraight() { 
  leftmotor.write(150);
  rightmotor.write(30);
}

void driveTurnCenterRight() {

  leftmotor.write(150);
  rightmotor.write(150);
}

void driveTurnCenterLeft() {

  leftmotor.write(30);
  rightmotor.write(30);
}

void driveBack() { //Line follow backwards
  if ((analogRead(lineBackL) > black) && (analogRead(lineBackR) < black)) {
    leftmotor.write(90);
    rightmotor.write(180);
  }
  else if ((analogRead(lineBackR) > black) && (analogRead(lineBackL) < black)) {
    leftmotor.write(0);
    rightmotor.write(90);
  }
  else {
    leftmotor.write(0);
    rightmotor.write(178);
  }
}

void driveBackSlow() {
  leftmotor.write(60);
  rightmotor.write(125);
}

void driveLeft() {
  leftmotor.write(90);
  rightmotor.write(0);
}

void driveRight() {
  leftmotor.write(180);
  rightmotor.write(90);
}

void stopMotors() {
  leftmotor.write(90);
  rightmotor.write(90);
}

void drive(double speed, Servo motor) {
  if (speed > 90) speed = 90;
  if (speed < -90) speed = -90;
  motor.write(90 + speed);
}
void turnAround() { //Turns around with front line sensor as end point
  switch (turnAroundState) {
    case setDesiredTime:
      desiredTime = millis() + 1300;
      turnAroundState = backUp;
      break;
    case backUp:
      if (millis() < desiredTime) {
        driveBack();
      }
      else {
        stopMotors();
        desiredTime = millis() + 500;
        turnAroundState = skipLine;
      }
      break;
    case skipLine:
      if (millis() < desiredTime) {
        driveTurnCenterRight();
      }
      else {
        stopMotors;
        turnAroundState = findLine;
      }
      break;
    case findLine:
      if (analogRead(lineRight) < black) {
        driveTurnCenterRight();
      }
      else {
        stopMotors();
        turnAroundState = setDesiredTime;
        turnedAround = true;
      }
      break;
  }
}

void moveToIntersection (int line) { // skips the amount of lines specified
  if (n != line) {
    lineFollow();
    if (backOnWhite && (analogRead(lineCounter) > black)) {
      n++;
      backOnWhite = false;
    }
    else if (!backOnWhite && (analogRead(lineCounter) < black)) {
      backOnWhite = true;
    }
  }
  else {
    stopMotors();
    backOnWhite = true;
    n = 0;
    intersectionFound = true;
  }

}

void pickupFunction() { //Moves arm down, grips, then up
  switch (pickupState) {
    case armDownRun:
      if (analogRead(pot) < armDown) {
        setpoint = armDown;
        inputValue = analogRead(pot);
        pid.Compute();
        drive(-outputValue, arm);
        gripper.write(60);
      }
      else {
        stopMotors();
        desiredTime = millis() + 1500;
        arm.write(90);
        pickupState = grip;
        radRodFull = false;
        sendRad = true;
        msg.sendRadiation(radRodFull);
        analogWrite(LEDS, 20);
      }
      break;
    case grip:
      if (millis() < desiredTime) {
        gripper.write(180);
      }
      else {
        gripper.write(130);
        pickupState = armUp;
      }

      break;
    case armUp:
      if (analogRead(pot) > armBack) {
        setpoint = armBack;
        inputValue = analogRead(pot);
        pid.Compute();
        drive(-outputValue, arm);
      }
      else {
        arm.write(90);
        pickupState = turnAroundRun;
      }
      break;
    case turnAroundRun:
      if (!turnedAround) {
        turnAround();
      }
      else {
        runState = navigateDump;
        msg.sendRobotStatus(0x02, 0x03, 0x04);
        pickupState = armDownRun;
        turnedAround = false;
      }
      break;
  }
}

void navigateDumpFunction() { //Go to nearest open supply area
  switch (navigateDumpState) {
    case findIntersection:


      if (!intersectionFound) {

        moveToIntersection(desiredDump);
      }
      else {
        intersectionFound = false;
        desiredTime = millis() + 300;
        navigateDumpState = turnToDump;
      }
      break;
    case turnToDump:
      if (!turned90) {
        if (firstReactor == true)
          turn90Left();
        else {
          turn90Right();

        }


      }
      else {
        navigateDumpState = approachDump;
        turned90 = false;
        desiredTime = millis() + 7000;
      }
      break;
    case approachDump:
      lineFollow();
      if ((digitalRead(bumpSwitch) == LOW) || (millis() > desiredTime) ) {
        stopMotors();
        navigateDumpState = findIntersection;

        runState = returnRod;
      }
      break;

  }

}

void turn90Left() {
  if (millis() < desiredTime) {
    driveTurnCenterLeft();
  }
  else if (analogRead(lineLeft) < black) {
    driveTurnCenterLeft();
  }
  else {
    stopMotors();
    turned90 = true;
  }
}

void turn90Right() {
  if (millis() < desiredTime) {
    driveTurnCenterRight();
  }
  else if (analogRead(lineRight) < black) {
    driveTurnCenterRight();
  }
  else {
    stopMotors();
    turned90 = true;
  }
}

void bluetoothSetBooleans() { //function for converting bluetooth 
  int supVal = (int)msg.getSupply();
  //Serial.print("supply");
  //Serial.println(supVal);
  if ((supVal % 2) == 0) {
    if (firstReactor)
      supply4 = false;
    else
      supply1 = false;
  }

  if (((supVal / 2) % 2) == 0) {
    if (firstReactor)
      supply3 = false;
    else
      supply2 = false;
  }

  if ((supVal > 7 && supVal < 12) || supVal < 4) {
    if (firstReactor)
      supply2 = false;
    else
      supply3 = false;
  }

  if (supVal < 8) {
    if (firstReactor)
      supply1 = false;
    else
      supply4 = false;
  }
  int dumpVal = (int)msg.getDump();
  //Serial.print("dump");
  //Serial.println(dumpVal);
  if ((dumpVal % 2) == 0) {
    if (firstReactor)
      dump4 = true;
    else
      dump1 = true;
  }

  if (((dumpVal / 2) % 2) == 0) {
    if (firstReactor)
      dump3 = true;
    else
      dump2 = true;
  }

  if ((dumpVal > 7 && dumpVal < 12) || dumpVal < 4) {
    if (firstReactor)
      dump2 = true;
    else
      dump3 = true;
  }

  if (dumpVal < 8) {
    if (firstReactor)
      dump1 = true;
    else
      dump4 = true;
  }

}

void setDesiredDumpAndSupply() { //sets variable for line skipping


  if (dump1 == true) {
    desiredDump = 1;
  }
  else if (dump2 == true) {
    desiredDump = 2;
  }
  else if (dump3 == true) {
    desiredDump = 3;
  }
  else if (dump4 == true) {
    desiredDump = 4;
  }

  if (supply1 == true) {
    desiredSupply = 1;
  }
  else if (supply2 == true) {
    desiredSupply = 2;
  }
  else if (supply3 == true) {
    desiredSupply = 3;
  }
  else if (supply4 == true) {
    desiredSupply = 4;
  }

}

void determineTurnsAndLineSkips() {
  if (desiredSupply == desiredDump) {
    dumpToSupplyTurn = 1;
  }
  else if (desiredSupply > desiredDump) {
    dumpToSupplyTurn = 0;
  }
  else {
    dumpToSupplyTurn = 2;
  }
  dumpToSupplyLineSkips = desiredSupply - desiredDump;
  absDumpToSupplyLineSkips = abs(dumpToSupplyLineSkips);
  runState = navigateReactor;
}

void returnRodFunction() {
  switch (returnRodState) {
    case armInsertRun:
      if (analogRead(pot) < armInsert) {
        setpoint = armInsert;
        inputValue = analogRead(pot);
        pid.Compute();
        drive(-outputValue, arm);

      }
      else {
        radRodFull = false;
        sendRad = false;
        analogWrite(LEDS, 0);
        stopMotors();
        desiredTime = millis() + 150;
        arm.write(90);
        returnRodState = releaseGrip;
      }
      break;
    case releaseGrip:
      if (millis() < desiredTime) {
        gripper.write(0);
        driveBack();

      }
      else {
        stopMotors();
        gripper.write(90);
        returnRodState = armBackRun;
      }

      break;
    case armBackRun:
      if (analogRead(pot) > armBack) {
        setpoint = armBack;
        inputValue = analogRead(pot);
        pid.Compute();
        drive(-outputValue, arm);
      }
      else {
        arm.write(90);
        returnRodState = turnAroundRunDump;
      }
      break;
    case turnAroundRunDump:
      if (!turnedAround) {
        turnAround();
      }
      else {
        runState = navigateNewRod;
        turnedAround = false;
        returnRodState = armInsertRun;
      }
      break;
  }
}
void navigateNewRodFunction() { //navigates to nearest supply
  switch (navigateNewRodState) {
    case navigateCenterLine:
      if (!intersectionFound) {
        moveToIntersection(1);
      }
      else {
        intersectionFound = false;
        navigateNewRodState = turnOnCenterLine;
        desiredTime = millis() + 300;
      }
      break;
    case turnOnCenterLine:

      if (!turned90) {
        if (dumpToSupplyTurn == 0) {
          if (firstReactor)
            turn90Left();
          else
            turn90Right();
        }
        else if (dumpToSupplyTurn == 2) {
          if (firstReactor)
            turn90Right();
          else
            turn90Left();
        }
        else {
          navigateNewRodState = approachSupply;
          desiredTime = millis() + 3500;
        }
      }
      else {
        turned90 = false;
        navigateNewRodState = moveToSupplyRow;
      }

      break;
    case moveToSupplyRow:
      if (!intersectionFound) {
        if (dumpToSupplyTurn == 0) {
          if (firstReactor)
            moveToIntersection(absDumpToSupplyLineSkips);
          else
            moveToIntersection(absDumpToSupplyLineSkips + 1);
        }
        else {
          if (firstReactor)
            moveToIntersection(absDumpToSupplyLineSkips + 1);
          else
            moveToIntersection(absDumpToSupplyLineSkips);

        }
      }
      else {
        intersectionFound = false;
        navigateNewRodState = turnToSupply;
        desiredTime = millis() + 300;
      }
      break;
    case turnToSupply:
      if (!turned90) {
        if (dumpToSupplyTurn == 2) {
          if (firstReactor)
            turn90Left();
          else
            turn90Right();
        }
        else if (dumpToSupplyTurn == 0) {
          if (firstReactor)
            turn90Right();
          else
            turn90Left();
        }
      }
      else {
        turned90 = false;
        navigateNewRodState = approachSupply;
        desiredTime = millis() + 3500;
      }
      break;
    case approachSupply:
      lineFollow();
      gripper.write(60);
      if ((digitalRead(bumpSwitch) == LOW) || (millis() > desiredTime) ) {
        stopMotors();
        navigateNewRodState = navigateCenterLine;
        gripper.write(90);

        runState = getNewRod;
      }
      break;
  }
}
void getNewRodFunction() {
  switch (getNewRodState) {
    case armRetrieveRun:
      if (analogRead(pot) < armPickupVal) {
        setpoint = armPickupVal;
        inputValue = analogRead(pot);
        pid.Compute();
        drive(-outputValue, arm);
      }
      else {
        stopMotors();
        desiredTime = millis() + 1200;
        arm.write(90);
        getNewRodState = closeGrip;
        radRodFull = true;
        sendRad = true;
        analogWrite(LEDS, 255);
        msg.sendRadiation(radRodFull);
      }
      break;
    case closeGrip:
      if (millis() < desiredTime) {
        gripper.write(180);
        //driveBack();

      }
      else {
        stopMotors();
        gripper.write(130);
        getNewRodState = armReverseRun;
      }
      break;
    case armReverseRun:
      if (analogRead(pot) > armBack) {
        setpoint = armBack;
        inputValue = analogRead(pot);
        pid.Compute();
        drive(-outputValue, arm);
      }
      else {
        arm.write(90);
        getNewRodState = turnAroundRunSupply;
      }
      break;
    case turnAroundRunSupply:
      if (!turnedAround) {
        turnAround();
      }
      else {
        runState = navigateSupplyToReactor;
        turnedAround = false;
        getNewRodState = armRetrieveRun;
      }
      break;
  }
}

void navigateSupplyToReactorFunction() {
  switch (navigateSupplyToReactorState) {
    case goToCenterLine:
      if (!intersectionFound) {
        moveToIntersection(1);
      }
      else {
        intersectionFound = false;
        navigateSupplyToReactorState = turnToReactor;
        desiredTime = millis() + 300;
      }
      break;
    case turnToReactor:
      if (!turned90) {
        if (firstReactor)
          turn90Left();
        else
          turn90Right();
      }
      else {
        turned90 = true;
        navigateSupplyToReactorState = moveToReactor;
      }
      break;
    case moveToReactor:
      lineFollow();
      if (digitalRead(bumpSwitch) == LOW) {
        stopMotors();
        runState = putDown;
        navigateSupplyToReactorState = goToCenterLine;
        desiredTime = millis() + 500;
      }
      break;
  }
}

//fun led stuff
void pulse(){
  if(brightness > 255 || brightness < 0)
    brightness = 100;
  if(millis() > ledTime+ledLength){
    if(brightness >= 255){
      up = false;
    }
    else if(brightness <= 0){
      up = true;
    }
    if(up){
      brightness += 4;
    }

    else{
      brightness -= 4;
    }
    ledTime = millis();
  }
}
void flash(){
  if(brightness != 255 && brightness != 0){
    brightness = 255;
  }
  if(millis() > (ledTime+ledFlashLength)){
    if(brightness == 255){
      brightness = 0;
    }
    else{
      brightness = 255;
    }
    ledTime = millis();
  }

}

//correctDistance, moveArmDown, releaseGripper, moveArmUp, turnAroundAfterDrop
void putDownFunction() {
  switch (putDownState) {
    case wait:
      if (millis() < desiredTime) {

      }
      else {

        putDownState = correctDistance;
        desiredTime = millis() + 100;
      }
      break;
    case correctDistance:
      if (digitalRead(bumpSwitch) == LOW) {
        leftmotor.write(69);
        rightmotor.write(117);
      }
      else {
        stopMotors();
        putDownState = moveArmDown;
      }
      break;
    case moveArmDown:
      if (analogRead(pot) < armInsertReactor) {

        setpoint = armDown;
        inputValue = analogRead(pot);
        pid.Compute();
        drive(-outputValue, arm);

      }
      else {
        stopMotors();
        desiredTime = millis() + 1500;
        arm.write(90);
        putDownState = releaseGripper;
        radRodFull = true;
        sendRad = false;
        analogWrite(LEDS, 0);
      }
      break;
    case releaseGripper:
      if (millis() < desiredTime) {
        gripper.write(0);
      }
      else {
        gripper.write(90);
        putDownState = moveArmUp;
      }
      break;
    case moveArmUp:
      if (analogRead(pot) > armBack) {
        setpoint = armBack;
        inputValue = analogRead(pot);
        pid.Compute();
        drive(-outputValue, arm);
      }
      else {
        arm.write(90);
        putDownState = turnAroundAfterDrop;
      }
      break;
    case turnAroundAfterDrop:
      if (!turnedAround) {
        turnAround();
      }
      else {
        if (firstReactor) {
          //reset all states for reactor B
          firstReactor = false;
          dump1 = false;
          dump2 = false;
          dump3 = false;
          dump4 = false;
          supply1 = true;
          supply2 = true;
          supply3 = true;
          supply4 = true;
          turnedAround = false;

          runState = determineBluetooth;
          pickupState = armDownRun;
          turnAroundState = setDesiredTime;
          navigateDumpState = findIntersection;
          returnRodState = armInsertRun;
          navigateNewRodState = navigateCenterLine;
          getNewRodState = armRetrieveRun;
          navigateSupplyToReactorState = goToCenterLine;
          putDownState = wait;
          backOnWhite = true;
          turnedAround = false;
          grabbed = false;
          intersectionFound = false;
          turned90 = false;
          n = 0;
        }
        else {
          runState = finished;
        }
        turnedAround = false;
        intersectionFound = false;

        returnRodState = armInsertRun;
      }
      break;
  }
}

