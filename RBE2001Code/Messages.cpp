/*
 * Messages.cpp
 *
 *  Created on: Sep 15, 2016
 *      Author: bradmiller
 */
#include "Arduino.h"
#include "Messages.h"
#include "BTComms.h"

BTComms comms;

/**
 * Constuctor
 * Initialize everything here when the class is created
 * Note: you cannot call methods that depend on other classes having already been created
 */
Messages::Messages() {
	stopped = true;
}

/**
 * Setup class code that is called from the Arduino sketch setup() function. This doesn't
 * get called until all the other classes have been created.
 */
void Messages::setup() {
	comms.setup();
}

/**
 * Check if the field is currently in the "stop" state
 * @returns bool value that is true if the robot should be stopped
 */
bool Messages::isStopped() {
	return stopped;
}

void Messages::setStopped(bool set){
  stopped = set;
}

/**
 * Send a heartbeat message to the field to let it know that your code is alive
 * This should be called by your robot program periodically, say once per second. This
 * timing can easily be done in the loop() function of your program.
 */
void Messages::sendHeartbeat() {
	comms.writeMessage(kHeartbeat, 0x08, 0x00);
}

void Messages::sendRadiation(bool f){
  if(f){
    comms.writeDataMessage(kRadiationAlert, 0x08, 0x00,0xFF);
  }
  else{
    comms.writeDataMessage(kRadiationAlert, 0x08, 0x00,0x2C);
  }
  
}

void Messages::sendRobotStatus(byte s1, byte s2, byte s3){
  comms.writeDataMessage3(kRobotStatus, 0x08, 0x00,s1,s2,s3);
}

/**
 * Print message for debugging
 * This method prints the message as a string of hex numbers strictly for debugging
 * purposes and is not required to be called for any other purpose.
 */
void Messages::printMessage() {
    for (int i = 0; i < comms.getMessageLength(); i++) {
      Serial.print(comms.getMessageByte(i), HEX);
      Serial.print(" ");
    }
    Serial.println();
}

void Messages::updateFieldMem(){
  currentByte = comms.getMessageByte(0);

  switch (currentByte){
    case 1:
      dump = comms.getMessageByte(3);
      break;
    case 2:
      supply = comms.getMessageByte(3);
      break;
    case 4:
      if(comms.getMessageByte(2)==0x00){
      stopped = true;
      }
      break;
    case 5:
      if(comms.getMessageByte(2)==0x00){
      stopped = false;
      }
      break;
  }
}

byte Messages::getSupply(){
  return supply;
}

byte Messages::getDump(){
  return dump;
}

/**
 * Read messages from the Bluetooth serial connection
 * This method should be called from the loop() function in your arduino code. It will check
 * to see if the lower level comms object has received a complete message, and run the appropriate
 * code to handle the message type. This should just save the state of the message inside this class
 * inside member variables. Then add getters/setters to retrieve the status from your program.
 */
bool Messages::read() {
	if (comms.read()) {
		switch (comms.getMessageByte(0)) {
		case kStorageAvailability:
			break;
		case kSupplyAvailability:
			break;
		case kRadiationAlert:
			break;
		case kStopMovement:
			break;
		case kResumeMovement:
			break;
		case kRobotStatus:
			break;
		case kHeartbeat:
			break;
		}
		return true;
	}
	return false;
}


