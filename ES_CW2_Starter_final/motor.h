#ifndef _motor_h_
#define  _motor_h_
#include "mbed.h"
#include "Crypto.h"
#include "mine.h"
#include "outgoingcomms.h"
#include "incomingcomms.h"
#include <iostream>

//Photointerrupter input pins
#define I1pin D3
#define I2pin D6
#define I3pin D5

//Incremental encoder input pins
#define CHApin   D12
#define CHBpin   D11

//Motor Drive output pins   //Mask in output byte
#define L1Lpin D1           //0x01
#define L1Hpin A3           //0x02
#define L2Lpin D0           //0x04
#define L2Hpin A6          //0x08
#define L3Lpin D10           //0x10
#define L3Hpin D2          //0x20

#define PWMpin D9

//Motor current sense
#define MCSPpin   A1
#define MCSNpin   A0
//Mapping from sequential drive states to motor phase outputs
/*
State   L1  L2  L3
0       H   -   L
1       -   H   L
2       L   H   -
3       L   -   H
4       -   L   H
5       H   L   -
6       -   -   -
7       -   -   -
*/

extern volatile int32_t motorPosition;

extern void motorCtrlFn();
extern void makeNoteVector(string tuneStr);
extern int tuneStep;

float motorRotationController();
float velControl();

extern Thread motorCtrlT;


#endif