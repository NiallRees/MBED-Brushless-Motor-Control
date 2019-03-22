#include "incomingcomms.h"
#include "motor.h"
#include "mbed.h"
#include <vector>


using namespace std; 

Queue<void, 8> inCharQ;

struct notePWM {
  string note; // this represents the decoded character
  int pwm; // the represents the morse encoding
};

vector<int> noteVector;
int tuneStep = 0;
float tunePWM = 0.0;

const notePWM NOTE_TABLE[] = {
    { "A", 1136 },
    { "A#", 1083 },
    { "Bb", 1083 },
    { "B", 1012 },
    { "C", 956 },
    { "C#", 902 },
    { "Db", 902 },
    { "D", 851 },
    { "D#", 804 },
    { "Eb", 804 },
    { "E", 758 },
    { "F", 716 },
    { "F#", 676 },
    { "Gb", 676 },
    { "G", 638 },
    { "G#", 602 },
    { "Ab", 602 }
};          

char buffer[17];
int bufferCount = 0;
uint64_t recievedkey;
Mutex newKey_mutex;
volatile bool new_key_set = false;
volatile bool playTune = false;
Timer t_timer;
float timer_time;
DigitalOut L23(L23pin);

volatile float target_velocity = 50.0;
volatile float act_rotations = 0.0;
volatile float tar_rotations = 0.0;
volatile float tar_rotations_tmp = 0.0;
string tuneStr;
char tuneCStr[40] = "";
int strLength = 0;

void motorTunePWM(){
    tunePWM = noteVector[tuneStep];
    if(tuneStep < noteVector.size() - 1) {
        tuneStep++;
    } else {
        tuneStep = 0;
    }
}

int chooseFreq(string note){
    for(int n = 0; n < 16; n++) {
        if(note == NOTE_TABLE[n].note) {
            return NOTE_TABLE[n].pwm;
        }
    }
}

void makeNoteVector(){
    noteVector.clear();
    for(int n = 0; n < strLength; n++) {
        if((tuneCStr[n+1] == '#') || (tuneCStr[n+1] == 'b')) {
            for(int i = 0; i < tuneStr[n+2]; i++){
                noteVector.push_back(chooseFreq(string(1, tuneCStr[n]) + string(1, tuneCStr[n+1])));
            }
            n = n + 2;
        } else  {
            for(int i = 0; i < (tuneCStr[n+1] - '0'); i++){
                noteVector.push_back(chooseFreq(string(1, tuneCStr[n])));
            }
            n = n + 1;
        }
    }
}

void serialISR(){

    uint8_t newChar = pc.getc();
    inCharQ.put((void*)newChar);

}

void recieve(){
    pc.attach(&serialISR);
    while(1) {
        osEvent newEvent = inCharQ.get();
        uint8_t newChar = (uint8_t)newEvent.value.p;
    
        if(bufferCount > 18)
            bufferCount = 0;
        else
            buffer[bufferCount]= newChar;
            
        if(newChar == '\r') {
            buffer[bufferCount] = '\0';
            bufferCount = 0;
            switch(buffer[0]){
            //KEY
            case 'K':    
                    newKey_mutex.lock();
                    sscanf(buffer,"K%x",&recievedkey);
                    newKey_mutex.unlock();
                    putMessage(KEY_1,(uint32_t)((recievedkey>>32)&0xFFFFFFFF));
                    putMessage(KEY_2,(uint32_t)(recievedkey&0xFFFFFFFF));
                    new_key_set = true;
                    break;
            // ROTATIONS
            case 'R':
                    sscanf(buffer, "R%f", &tar_rotations_tmp);
                    act_rotations = 0;
                    tar_rotations = ((float)motorPosition)/6 + tar_rotations_tmp;
                    break;
            // VELOCITY
            case 'V':
                    sscanf(buffer, "V%f", &target_velocity);
                    break;
            // TUNE
            case 'T':
                    sscanf(buffer, "T%s", tuneStr);
                    strcpy(tuneCStr, tuneStr.c_str());
                    strLength = strlen(tuneCStr);
                    playTune = false;
                    makeNoteVector();
                    playTune = true;
                    putMessage(PLAY_TUNE, 0);
                    break;
            default:
                break;
            }
        } else {
                bufferCount++;
        }
    }
}