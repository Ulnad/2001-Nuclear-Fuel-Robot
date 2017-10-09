#include <Servo.h>
#include <PID_v1.h>


Servo rightmotor; // Servo object
Servo leftmotor; // Servo object
Servo arm;
Servo gripper;

bool backOnWhite = true;
bool turnedAround = false;
bool grabbed = false;
bool intersectionFound = false;
bool turned90 = false;

int n;
int desiredDump = 4;
int lineCounter = A0;
int lineLeft = A1;
int lineRight = A2;
int lineBack = A3;
int pot = A4;
int  bumpSwitch = 29;
int black = 700;
int armBack = 38;
int armDown = 385;
int Kp = 1000;
int Ki = 0;
int Kd = 0;
long desiredTime;
double setpoint, inputValue, outputValue;
PID pid(&inputValue, &outputValue, &setpoint, Kp, Ki, Kd, DIRECT);

static enum runStates {decideNextState, determineBluetooth, navigateReactor, navigateDump, navigateNewRod, pickUp, putDown, returnRod, getNewRod, finished}
runState;
static enum turnAroundStates {setDesiredTime, backUp, skipLine, findLine}
turnAroundState;
static enum pickupStates {armDownRun, grip, armUp, turnAroundRun}
pickupState;
static enum navigateDumpStates {findIntersection, turnToDump, approachDump}
navigateDumpState;


void setup() {
  // put your setup code here, to run once:
  pinMode(bumpSwitch, INPUT_PULLUP);
  leftmotor.attach(11, 1000, 2000); // left drive motor pin#, pulse time for 0,pulse time for 180
  rightmotor.attach(10, 1000, 2000); // right drive motor pin#, pulse time for 0,pulse time for 180
  arm.attach(9, 1000, 2000);
  gripper.attach(4, 1000, 2000);
  Serial.begin(9600);
  runState = navigateReactor;
  pickupState = armDownRun;
  turnAroundState = setDesiredTime;

  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-90, 90);

}

void runRobot() {
  switch (runState) {
    case decideNextState:

      break;
    case determineBluetooth:

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

      break;
    case pickUp:
      pickupFunction();
      break;
    case putDown:

      break;
    case returnRod:

      break;
    case getNewRod:

      break;
    case finished:

      break;
  }

}
void loop() {
  runRobot();
  //moveToIntersection(4);
  Serial.println(analogRead(pot));

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
void turnAround() {
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
        gripper.write(0);
      }
      else {
        stopMotors();
        desiredTime = millis() + 1000;
        arm.write(90);
        pickupState = grip;
      }
      break;
    case grip:
      if (millis() < desiredTime) {
        gripper.write(180);
      }
      else {
        gripper.write(90);
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
      }
      break;
    case approachDump:
      lineFollow();
      if (digitalRead(bumpSwitch) == LOW) {
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


