#ifndef _mine_h_
#define  _mine_h_

#include "Crypto.h"
#include "mbed.h"
//#include "extcomms.h"

extern uint8_t sequence[];
extern uint64_t* key;
extern uint64_t* nonce;
extern uint8_t hash[32];
extern float hashes;

extern void computeHash();

#endif