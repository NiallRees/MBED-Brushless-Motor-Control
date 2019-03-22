#include "mbed.h"
#include "mine.h"
#include "motor.h"
#include "outgoingcomms.h"
#include "incomingcomms.h"
#include "rtos.h"

Thread incoming(osPriorityNormal,1536);
Thread outgoing(osPriorityNormal,1536);
Thread motorCtrlT(osPriorityHigh,1024);

int main(){
    
    outgoing.start(commOutFn);
    incoming.start(recieve);
    motorCtrlT.start(motorCtrlFn);
    
     while (1) {
        computeHash();
    }
    
}