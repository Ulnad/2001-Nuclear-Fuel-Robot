#include <Servo.h>

Servo rightmotor; // Servo object
Servo leftmotor; // Servo object
Servo arm;
Servo gripper;
int n;
int lineCounter = A0;
int lineLeft = A1;
int lineRight = A2;
int lineBack = A3;
int pot = A4;
int  bumpSwitch = 22;
int black = 400;

static enum runStates {decideNextState, determineBluetooth, navigateReactor, navigateDump, navigateNewRod, pickUp, putDown, returnRod, getNewRod, finished}
runState;


void setup() {
  // put your setup code here, to run once:
  pinMode(bumpSwitch, INPUT);
  leftmotor.attach(11, 1000, 2000); // left drive motor pin#, pulse time for 0,pulse time for 180
  rightmotor.attach(10, 1000, 2000); // right drive motor pin#, pulse time for 0,pulse time for 180
  arm.attach(9, 1000, 2000);
  gripper.attach(4, 1000, 2000);
}

void runRobot() {
  switch (runState) {
    case decideNextState:

      break;
    case determineBluetooth:

      break;
    case naviateReactor:

      break;
      case navigateDump:

      break;
      case navigateNewRod:

      break;
      case pickUp:

      break;
      case putDown:

      break
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
}

void lineFollow(){
  if
}

