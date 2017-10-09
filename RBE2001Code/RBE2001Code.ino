#include <Servo.h>
#include <PID_v1.h>


Servo rightmotor; // Servo object
Servo leftmotor; // Servo object
Servo arm;
Servo gripper;

bool turnedAround = false;
bool grabbed = false;
int n;
int lineCounter = A0;
int lineLeft = A1;
int lineRight = A2;
int lineBack = A3;
int pot = A4;
int  bumpSwitch = 29;
int black = 700;
int armBack = 26;
int armDown = 385;
int Kp = 1000;
int Ki = 0;
int Kd = 0;
long desiredTime;
double setpoint, inputValue, outputValue;
PID pid(&inputValue, &outputValue, &setpoint, Kp, Ki, Kd, DIRECT);

static enum runStates {decideNextState, determineBluetooth, navigateReactor, navigateDump, navigateNewRod, pickUp, putDown, returnRod, getNewRod, finished}
runState;
static enum turnAroundStates {backUp, skipLine, findLine}
turnAroundState;
static enum pickupStates {armDownRun, grip, armUp, turnAroundRun}
pickupState;


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
  turnAroundState = backUp;

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
void driveTurnCenter() {

  leftmotor.write(0);
  rightmotor.write(180);
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
    case backUp:
      if (millis < desiredTime) {
        driveBack();
      }
      else {
        stopMotors();
        desiredTime = millis() + 300;
        turnAroundState = skipLine;
      }
      break;
    case skipLine:
      if (millis < desiredTime) {
        driveTurnCenter();
      }
      else {
        stopMotors;
        turnAroundState = findLine;
      }
      break;
    case findLine:
      if (analogRead(lineLeft) < black) {
        driveTurnCenter();
      }
      else {
        stopMotors();
        turnAroundState = backUp;
        turnedAround = true;
      }
      break;
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
        stopMotors();
        desiredTime = millis() + 300;
pickupState = turnAroundRun;
      }
      break;
    case turnAroundRun:
if (!turnedAround) {
         turnAround();
       }
       else{
        runState = navigateDump;
        pickupState = armDownRun;
       }
      break;
  }

  /* if ((analogRead(pot) < armDown) && (!grabbed)) {
         setpoint = armDown;
         inputValue = analogRead(pot);
         pid.Compute();
         drive(-outputValue, arm);
         desiredTime = millis() + 1000;
         gripper.write(0);
       }
       else if (millis() < desiredTime) {
         arm.write(90);
         gripper.write(180);
         grabbed = true;
       }
       else if (analogRead(pot) > armBack) {
         gripper.write(90);
         setpoint = armBack;
         inputValue = analogRead(pot);
         pid.Compute();
         drive(-outputValue, arm);
         desiredTime = millis() + 500;

       }
       else if (!turnedAround) {
         turnAround();
       }

       else {
         runState = navigateDump;
         turnedAround = false;
       }
  */
}

