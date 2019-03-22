#include "outgoingcomms.h"
#include "mbed.h"

// Mail FIFO queue
Mail<message_t, 16> outMessages;


// Places messages in mail queue
void putMessage(uint8_t code, int32_t data){
    message_t *pMessage = outMessages.alloc();
    pMessage->code = code;
    pMessage->data = data;
    outMessages.put(pMessage);
}

RawSerial pc(SERIAL_TX, SERIAL_RX);

// This thread checks for new messages and prints them
void commOutFn(){
    while(1) {
        osEvent newEvent = outMessages.get();
        message_t *pMessage = (message_t*)newEvent.value.p;
        
        switch(pMessage->code) {
        case (NONCE_1):
                pc.printf("Nonce: 0x%x", pMessage->data);
                break;
        case (NONCE_2):
                pc.printf("%x\r\n", pMessage->data);
                break;
        case (KEY_1):
                pc.printf("New Key:  0x%x", pMessage->data);
                break;
        case (KEY_2):
                pc.printf("%x\r\n", pMessage->data);
                break;
        case (VELOCITY):
                pc.printf("Motor Velocity: %f rev/s\r\n", *(float*)&(pMessage->data));
                break;
        case (TAR_VELOCITY):
                pc.printf("Target Velocity: %f rev/s\r\n", *(float*)&(pMessage->data));
                break;
        case (ROTATION):
                pc.printf("Actual rotations: %f \r\n", *(float*)&(pMessage->data));
                break;
        case (TAR_ROTATION):
                pc.printf("Target Rotation: %f \r\n", *(float*)&(pMessage->data));
                break;
        case (PLAY_TUNE):
                pc.printf("Tune toggled \r\n");
                break;
        case (TEST):
                pc.printf("PWM us: %f \r\n", *(float*)&(pMessage->data));
                break;
        default:
                pc.printf("Unknown code, data: %d \r\n", pMessage->data);
                break;
        }
        outMessages.free(pMessage);
    }
}
        