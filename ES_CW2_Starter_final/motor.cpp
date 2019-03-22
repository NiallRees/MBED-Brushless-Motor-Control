#include "motor.h"
#include "mine.h"

//Motor Drive Table
const int8_t driveTable[] = {0x12,0x18,0x09,0x21,0x24,0x06,0x00,0x00};

//Hot Cross Buns Tune
//const int TUNE[] = {759, 851, 955, 955, 759, 851, 955, 955, 955, 955, 851, 851, 759, 851, 955, 955};

float tunePeriod = 1;

//Mapping from interrupter inputs to sequential rotor states. 0x00 and 0x07 are not valid
const int8_t stateMap[] = {0x07,0x05,0x03,0x04,0x01,0x00,0x02,0x07};  
//const int8_t stateMap[] = {0x07,0x01,0x03,0x02,0x05,0x00,0x04,0x07}; 
//Alternative if phase order of input or drive is reversed

//Phase lead to make motor spin
int8_t lead = 2;  //2 for forwards, -2 for backwards

//Status LED
DigitalOut led1(LED1);

//Photointerrupter inputs
InterruptIn I1(I1pin);
InterruptIn I2(I2pin);
InterruptIn I3(I3pin);

//Motor Drive outputs
DigitalOut L1L(L1Lpin);
DigitalOut L1H(L1Hpin);
DigitalOut L2L(L2Lpin);
DigitalOut L2H(L2Hpin);
DigitalOut L3L(L3Lpin);
DigitalOut L3H(L3Hpin);
PwmOut PWM_pin(PWMpin);


/***************** Global Variables ******************************/
int velocitycount = 0;
float velocity = 0.0;
float velocityoverten = 0.0;
float prev_tar_rotations  = 0.0;
volatile int32_t motorPosition = 0;
int revstimes6 = 0;
int revstimes6old = 0;
int8_t orState = 0; 
int8_t intState = 0;
int8_t oldState = 0;
int8_t differenceState = 0;
int print_count = 0;
int prev_tar_velocity = 0;
float motorPWM = 0.0;
float motorPWM_vel = 0.0;
float motorPWM_rot = 0.0;
float velocityError = 0.0;
float differentialVelocityError = 0.0;
int PWM_LIMIT = 2000;
Timer t;
float veltime = 0;
float oldHashes = 0;
float hashRate = 0;

// PID velocity gains
#define V_P     0.1
#define V_I     0.1
#define V_D     0.0
#define V_I_MAX   1

// PID rotation gains
#define R_P 0.01
#define R_I     4
#define R_D     0.21
#define R_I_MAX   0.42

// PID error variables
float integralVelocityError = 0;
float prevVelocityError = 0;
float integralRotationsError = 0;
float prevRotationError = 0;
float differentialRotationError = 0;

float previous_rot_velocity     = 0.0;
float previous_vel_velocity     = 0.0;

uint32_t controlspeed();

//Set a given drive state
void motorOut(int8_t driveState){
    
    //Lookup the output byte from the drive state.
    int8_t driveOut = driveTable[driveState & 0x07];
      
    //Turn off first
    if (~driveOut & 0x01) L1L = 0;
    if (~driveOut & 0x02) L1H = 1;
    if (~driveOut & 0x04) L2L = 0;
    if (~driveOut & 0x08) L2H = 1;
    if (~driveOut & 0x10) L3L = 0;
    if (~driveOut & 0x20) L3H = 1;
    
    //Then turn on
    if (driveOut & 0x01) L1L = 1;
    if (driveOut & 0x02) L1H = 0;
    if (driveOut & 0x04) L2L = 1;
    if (driveOut & 0x08) L2H = 0;
    if (driveOut & 0x10) L3L = 1;
    if (driveOut & 0x20) L3H = 0;
    }
    
    //Convert photointerrupter inputs to a rotor state
inline int8_t readRotorState(){
    return stateMap[I1 + 2*I2 + 4*I3];
    }

//Basic synchronisation routine    
int8_t motorHome() {
    //Put the motor in drive state 0 and wait for it to stabilise
    motorOut(0);
    PWM_pin.write(1.0f);
    wait(2.0);
    
    //Get the rotor state
    return readRotorState();
}
/************************** Functions ******************************/

void motorCtrlTick(){
    motorCtrlT.signal_set(0x1);
}    

float velControl(){
    float T_s;
    
    lead = (target_velocity<0) ? -2 : 2;
    
    //Proportional calculation
    velocityError = abs(target_velocity) - abs(velocity);
    
    //Differential calculation
    differentialVelocityError = velocityError - prevVelocityError;
    prevVelocityError = velocityError;
    
    // Integral calculation
    integralVelocityError += velocityError*V_I*veltime;
    if(integralVelocityError > V_I_MAX) integralVelocityError = V_I_MAX;
    if(integralVelocityError < 0) integralVelocityError =0.0;
    
    //Duty cycle calculation with constants
    T_s = V_P*(abs(target_velocity) - abs(velocity)) + integralVelocityError + V_D*differentialVelocityError; 

    // Ensure T_s is positive
    T_s = (T_s>0) ? T_s : 0;

    // Prevent exceeding 1.0 duty cycle
    T_s = (T_s > 1.0) ? 1.0 : T_s;

    // Keep duty cycle over 0.25 to prevent undershoot
    T_s = T_s ? T_s : 0.25;
    return T_s; 
}

// Implement the rotation control
float motorRotationController(){
        float T_r;

        //Proportional calculation
        float rotationError = tar_rotations - act_rotations;

        // Differential calculation
        differentialRotationError = rotationError - prevRotationError;

        // Integral calculation
        integralRotationsError += rotationError*R_I*veltime;
        if(integralRotationsError > R_I_MAX) integralRotationsError    = R_I_MAX;
        if(integralRotationsError < (-R_I_MAX)) integralRotationsError =-R_I_MAX;

        //Duty cycle calculation with constants
        T_r = R_P*(rotationError) + integralRotationsError + R_D*differentialRotationError;

        // Change direction if overshoot
        lead = (T_r > 0) ?  2 : -2;
        T_r = abs(T_r);
        
        // Prevent exceeding 1.0 duty cycle
        T_r = (T_r > 1.0) ? 1.0 : T_r;
        
        // If error is within 1 of the required rotations, stop rotating
        if((prevRotationError < 1) && (prevRotationError == rotationError)) {T_r = 0;}
        prevRotationError = rotationError;

        return T_r; 
}


/***************** Interrupt Service Routines **********************/
void ISR_rotor(){
    intState = readRotorState();
    differenceState = intState - oldState;
    if(differenceState == 5) revstimes6--;
    else if (differenceState == -5) revstimes6++;
    else revstimes6 += intState - oldState;
    
    motorOut((intState-orState+lead+6)%6); // positive remainder by adding 6
    oldState = intState;
    }
    
//Main function
void motorCtrlFn() {
    PWM_pin.period_ms(2);
    PWM_pin.write(1.0);
    
    //Run the motor synchronisation
    orState = motorHome();
    //orState is subtracted from future rotor state inputs to align rotor and motor states

    //attach interrupts 
    I1.rise(&ISR_rotor);
    I1.fall(&ISR_rotor);
    I2.rise(&ISR_rotor);
    I2.fall(&ISR_rotor);
    I3.rise(&ISR_rotor);
    I3.fall(&ISR_rotor);
    
    Ticker motorCtrlTicker;
    motorCtrlTicker.attach_us(&motorCtrlTick,100000);
    
    //Poll the rotor state and set the motor outputs accordingly to spin the motor
    while(1){
        t.stop();
        veltime = t.read();
        t.reset();
        t.start();
        //If velocity or rotations have changed we need to reset the integral limits
        if (tar_rotations != prev_tar_rotations) {
                        integralRotationsError = 0;
                        prev_tar_rotations = tar_rotations;
        } else if (target_velocity != prev_tar_velocity ) {
                        integralVelocityError  = 0;
                        prev_tar_velocity  = target_velocity;
        } 
        
        motorCtrlT.signal_wait(0x1);//Run loop every 100ms
        core_util_critical_section_enter();
        velocity = (revstimes6 - revstimes6old)*10/6;
        act_rotations += velocity/10;
        revstimes6old = revstimes6;
        core_util_critical_section_exit();
        velocitycount++;
        
        //If no target rotations are set then spin motor per velocity PID controller
        if(!tar_rotations) {
        motorPWM = velControl();
        PWM_pin = motorPWM;
        
        }
        //If current velocity is zero and there is a target velocity or rotations set
        if(velocity==0 && (target_velocity || tar_rotations)) {
            int8_t rotorState = readRotorState();
            int8_t tmpDriveState = (rotorState - orState + lead + 6);
            tmpDriveState = tmpDriveState % 6;
            motorOut(tmpDriveState);
        }
        // If target velocity is set to 0 spin as fast as possible
        if(target_velocity == 0) {
            PWM_pin = 1;
        } else if (target_velocity && tar_rotations) { //If target velocity and target rotations is set
                motorPWM_vel = velControl();
                motorPWM_rot = motorRotationController();
                if(lead == 2){
                    PWM_pin = (motorPWM_vel<motorPWM_rot) ? motorPWM_vel : motorPWM_rot;  // max 
                    }
                else{
                    PWM_pin = (motorPWM_vel>motorPWM_rot) ? motorPWM_vel : motorPWM_rot;  // min
                    } 
        }
        //This loop runs every second
        if(velocitycount == 10) {
            
            if(playTune) {
                motorTunePWM();
                PWM_pin.period_us(tunePWM);
            } else {
                PWM_pin.period_ms(2);
            }
            
            hashRate = hashes - oldHashes;
            oldHashes = hashes;
            
            
            putMessage(VELOCITY, *(int32_t*)&velocity);
            putMessage(TAR_VELOCITY,*(int32_t*)&target_velocity);
            putMessage(ROTATION, *(int32_t*)&act_rotations);
            putMessage(TAR_ROTATION,*(int32_t*)&tar_rotations);
            putMessage(TEST, *(int32_t*)&tunePWM);
            velocitycount = 0;
        }
    }
    

}
