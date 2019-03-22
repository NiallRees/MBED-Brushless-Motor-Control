#include "mine.h"
#include "outgoingcomms.h"
#include "incomingcomms.h" 

uint8_t sequence[] = {
    0x45,0x6D,0x62,0x65,0x64,0x64,0x65,0x64,
    0x20,0x53,0x79,0x73,0x74,0x65,0x6D,0x73,
    0x20,0x61,0x72,0x65,0x20,0x66,0x75,0x6E,
    0x20,0x61,0x6E,0x64,0x20,0x64,0x6F,0x20,
    0x61,0x77,0x65,0x73,0x6F,0x6D,0x65,0x20,
    0x74,0x68,0x69,0x6E,0x67,0x73,0x21,0x20,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
uint64_t* key = (uint64_t*)((int)sequence + 48);
uint64_t* nonce = (uint64_t*)((int)sequence + 56);
uint8_t hash[32];
extern float hashes = 0;

void computeHash(){
    newKey_mutex.lock();
    *key = recievedkey;
    newKey_mutex.unlock();
    
    SHA256::computeHash(&hash[0], &sequence[0], 64);
    hashes++;
    (*nonce)++;
        
    if(((hash[0])==0) && ((hash[1])==0)) {
        putMessage(NONCE_1, (uint32_t)((*nonce>>32)&0xFFFFFFFF));
        putMessage(NONCE_2, (uint32_t)(*nonce&0xFFFFFFFF));
    }
}