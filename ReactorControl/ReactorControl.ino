#include "Arduino.h"
#include "Messages.h"

Messages msg;
unsigned long timeForHeartbeat;
int initializeMem = 0;
int dataCounter = 0;
bool radRodFull;

/**
 * This "starter" program reads and debug-prints messages from the field. It also sends
 * heartbeat messages every second letting the field know that the robot is still running.
 *
 * You can use this as the basis for your project, or copy the elements from here into your
 * project. If you do that, be sure that you include the Messages and BTComms classes (.cpp
 * and .h files) into your new project.
 */

/**
 * Initialize the messages class and the debug serial port
 */
void setup() {
  Serial.begin(115200);
  Serial.println("Starting");
  msg.setup();
  timeForHeartbeat = millis() + 1000;
  radRodFull = true;
}

/**
 * The loop method runs continuously, so call the Messages read method to check for input
 * on the bluetooth serial port and parse any complete messages. Then send the heartbeat if
 * it's time to do that.
 *
 * For the final project, one good way of implementing it is to use a state machine with high
 * level tasks as states. The state will tell what you should be doing each time the loop
 * function is called.
 *
 * Refer to the state machine lecture or look at the BTComms class for an example on how to
 * implement state machines.
 */
void loop() {
  if (msg.read()) {
	  msg.updateFieldMem();
    if(initializeMem < 3){
      initializeMem++;
    }
    else{
      msg.printMessage();
      Serial.print("stopped:");
      Serial.print(msg.isStopped());
      Serial.print("  Dump:");
      Serial.println(msg.getDump());
    }
  }
  if (millis() > timeForHeartbeat) {
    timeForHeartbeat = millis() + 1000;
    msg.sendHeartbeat();
    if(dataCounter >= 4){
      dataCounter = 0;
      msg.sendRadiation(radRodFull);
      msg.sendRobotStatus(0x02,0x01,0x04);
    }
    else{
      dataCounter++;
    }
  }
}
