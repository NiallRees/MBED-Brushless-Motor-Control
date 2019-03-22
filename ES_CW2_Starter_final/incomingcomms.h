#ifndef _incomingcomms_h_
#define  _incomingcomms_h_
#define L23pin D4

#include "mine.h"
#include "outgoingcomms.h"
#include "mbed.h"

extern volatile float target_velocity;
extern volatile float act_rotations;
extern volatile float tar_rotations;
extern volatile bool playTune;

extern uint64_t recievedkey;
extern Mutex newKey_mutex;
extern volatile bool new_key_set;
extern float tunePWM;

void serialISR();
extern void recieve();
extern void motorTunePWM();

#endif