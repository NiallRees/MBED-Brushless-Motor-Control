#ifndef _outgoingcomms_h_
#define  _outgoingcomms_h_

#include "rtos.h"
#include "mbed.h"

#define NONCE_1             1
#define NONCE_2             2
#define KEY_1               3
#define KEY_2               4
#define VELOCITY            5
#define TAR_VELOCITY        6
#define ROTATION            7
#define TAR_ROTATION        8
#define PLAY_TUNE           9
#define TEST                10
#define ERROR               99

extern RawSerial pc;

typedef struct {
    uint8_t code;
    int32_t data;
} message_t;

extern void putMessage(uint8_t code, int32_t data);
extern void commOutFn();

#endif