#include <Servo.h>
#include <PID_v1.h>


Servo rightmotor; // Servo object
Servo leftmotor; // Servo object
Servo arm;
Servo gripper;

bool dump1 = false;
bool dump2 = false ;
bool dump3 = true ;
bool dump4 = true;
bool supply1 = true ;
bool supply2 = false;
bool supply3 = false;
bool supply4 = false;


bool backOnWhite = true;
bool turnedAround = false;
bool grabbed = false;
bool intersectionFound = false;
bool turned90 = false;

int dumpToSupplyTurn; //0 is left, 1 is no turn, 2 is right turn
int dumpToSupplyLineSkips;
int absDumpToSupplyLineSkips;
int n;
int desiredDump;
int desiredSupply;
int lineCounter = A0;
int lineLeft = A1;
int lineRight = A2;
int lineBack = A3;
int pot = A5;
int  bumpSwitch = 29;
int black = 700;
int armBack = 38;
int armDown = 380;
int armInsert = 115;
int armPickupVal = 55;
int Kp = 1000;
int Ki = 0;
int Kd = 0;
long desiredTime;
double setpoint, inputValue, outputValue;
PID pid(&inputValue, &outputValue, &setpoint, Kp, Ki, Kd, DIRECT);

static enum runStates {decideNextState, determineBluetooth, navigateReactor, navigateDump, navigateNewRod, navigateSupplyToReactor, pickUp, putDown, returnRod, getNewRod, finished}
runState;
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
static enum putDownStates {correctDistance, moveArmDown, releaseGripper, moveArmUp, turnAroundAfterDrop}
putDownState;


void setup() {
  // put your setup code here, to run once:
  pinMode(bumpSwitch, INPUT_PULLUP);
  leftmotor.attach(11, 1000, 2000); // left drive motor pin#, pulse time for 0,pulse time for 180
  rightmotor.attach(10, 1000, 2000); // right drive motor pin#, pulse time for 0,pulse time for 180
  arm.attach(9, 1000, 2000);
  gripper.attach(4, 1000, 2000);
  Serial.begin(9600);
  runState = determineBluetooth;
  pickupState = armDownRun;
  turnAroundState = setDesiredTime;
  navigateDumpState = findIntersection;
  returnRodState = armInsertRun;
  navigateNewRodState = navigateCenterLine;
  getNewRodState = armRetrieveRun;
  navigateSupplyToReactorState = goToCenterLine;

  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-90, 90);

}

void runRobot() {
  switch (runState) {
    case decideNextState:

      break;
    case determineBluetooth:

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
  //gripper.write(60);
  runRobot();
  //moveToIntersection(4);
  Serial.print(desiredSupply);
  Serial.print("    ");
  Serial.println(absDumpToSupplyLineSkips);

  //Serial.println(desiredDump);

}

void lineFollow() {
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
  leftmotor.write(180);
  rightmotor.write(0);
}
void driveTurnCenterRight() {

  leftmotor.write(180);
  rightmotor.write(180);
}
void driveTurnCenterLeft() {

  leftmotor.write(0);
  rightmotor.write(0);
}
void driveBack() {
  leftmotor.write(0);
  rightmotor.write(180);
}

void driveBackSlow() {
  leftmotor.write(60);
  rightmotor.write(120);
}

void driveLeft() {
  leftmotor.write(110);
  rightmotor.write(0);
}

void driveRight() {
  leftmotor.write(180);
  rightmotor.write(70);
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
void turnAround() {
  switch (turnAroundState) {
    case setDesiredTime:
      desiredTime = millis() + 1350;
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
void moveToIntersection (int line) {
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
void pickupFunction() {
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
      }
      break;
    case grip:
      if (millis() < desiredTime) {
        gripper.write(180);
      }
      else {
        gripper.write(120);
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
        pickupState = armDownRun;
        turnedAround = false;
      }
      break;
  }
}

void navigateDumpFunction() {
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
        turn90Left();
      }
      else {
        navigateDumpState = approachDump;
        turned90 = false;
        desiredTime = millis() + 4000;
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

void setDesiredDumpAndSupply() {


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
void navigateNewRodFunction() {
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
        if (dumpToSupplyTurn = 0) {
          turn90Left();
        }
        else if (dumpToSupplyTurn = 2) {
          turn90Right();
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
        moveToIntersection(absDumpToSupplyLineSkips + 1);
      }
      else {
        intersectionFound = false;
        navigateNewRodState = turnToSupply;
      }
      break;
    case turnToSupply:
      if (!turned90) {
        if (dumpToSupplyTurn = 2) {
          turn90Left();
        }
        else if (dumpToSupplyTurn = 1) {
          turn90Right();
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
      }
      break;
    case closeGrip:
      if (millis() < desiredTime) {
        gripper.write(180);
        //driveBack();

      }
      else {
        stopMotors();
        gripper.write(110);
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
        turn90Left();
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
      }
      break;
  }
}

//correctDistance, moveArmDown, releaseGripper, moveArmUp, turnAroundAfterDrop
void putDownFunction(){
  switch (putDownState){
    case correctDistance:

    break;
    case moveArmDown:

    break;
    case releaseGripper:

    break;
    case moveArmUp:

    break;
    case turnAroundAfterDrop:

    break;
  }
}

