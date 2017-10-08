#ifndef _BTReader
#define _BTReader

/**
 * Low level class to both receive and send message to the field.
 * The writeMessage() method sends messages to the field and the other methods
 * receive message from the field.
 *
 * This class is used by the higher level Messages class to separate the actual
 * byte reading and dealing with checksums from the messages class to make it more
 * understandable.
 */
class BTComms {
  public:
    BTComms();
    void setup();
    int getMessageLength();
    unsigned char getMessageByte(unsigned index);
    bool read();
    void writeMessage(unsigned char b1, unsigned char b2, unsigned char b3);
   private:
    enum BTstate {kLookingForStart, kReadingMessageLength, kReadMessage} BTstate;
    unsigned messageLength;
    static const int messageBufferLength = 20;
    unsigned char message[messageBufferLength];
    unsigned messageIndex;
    unsigned char kMessageStart = 0x5f;
};

#endif
