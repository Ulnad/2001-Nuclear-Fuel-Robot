/*
 * Messages.h
 *
 *  Created on: Sep 15, 2016
 *      Author: bradmiller
 */

#ifndef MESSAGES_H_
#define MESSAGES_H_

/**
 * Handles Bluetooth messages as they are received from the Reactor Control System
 * This class keeps the current state of the field and allows your program to query it at
 * any time. The read() method is called inside the loop() of your program. It reads a
 * message and sets the internal class state based on it's contents. This happens each loop.
 * You then write methods (like isStopped(), below) that report on that state. The idea is
 * that the receipt and parsing of the messages is decoupled from your use of the state data
 * making your programs much less complex.
 *
 * You need to add code and private state variables (like stopped, below) to parse and remember
 * the RCS state. Then add methods that can be called independent of message receipt to retrieve
 * that state.
 *
 * Other things you may want to do:
 * - check is messages are addressed to your robot (or broadcast to all)
 * - handle the source and destination fields which exist for all messages
 * - add methods to determine which storage is free/used so you can ask by number
 *   rather than having to do the bit mask stuff
 */
class Messages {
public:
	typedef enum {kReserved,
		kStorageAvailability,
		kSupplyAvailability,
		kRadiationAlert,
		kStopMovement,
		kResumeMovement,
		kRobotStatus,
		kHeartbeat} MessageType;

	Messages();

	bool isStopped();
	void sendHeartbeat();
	bool read();
	void setup();
	void printMessage();

private:
	bool stopped;
};

#endif /* MESSAGES_H_ */
