#include <Arduino.h>

#include "I2Cdev.h"
#include <PID_v1.h>

#include "MPU6050_6Axis_MotionApps20.h"

#include "motordriver.h"

MPU6050 mpu;

bool debug = true;

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define DEBUG_PIN 7
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successfulfw
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion quater;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

bool rcCheckStatus = false;
bool readyToFlyStatus = false;

const double  CoefP = 0.25;
const double  CoefI = 0.05;
const double  CoefD = 0.1;

// requested paarameters [0, 100]%
// double speedRequest = 0;
// double pitchRequest = 0;
// double rollRequest = 0;

// Input, Output, Setpoint.
PID pitchPID(&pitchInput, &pitchOutput, &setPitch, CoefP, CoefI, CoefD, DIRECT);
PID rollPID(&rollInput, &rollOutput, &setRoll, CoefP, CoefI, CoefD, DIRECT);

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

void readRcControllerData();
bool isShutDownRequested();
bool isMpuDataActual();
void shutDownQ();
void clearControlRequestValues();
void Generate_Motor_Pulses();
void Update_Motor_Values();

void dmpDataReady() 
{
    mpuInterrupt = true;

    //Generate pulses
    // Generate_Motor_Pulses();
}

uint64_t lastMpuReaded = 0;
const uint64_t MaxTimeWaitingForMpuData = 100;

int motor_upper_left_us = 0;
int motor_upper_right_us = 0;
int motor_lower_left_us = 0;
int motor_lower_right_us = 0;

void setup() 
{
    setupDriver();  // Init motors with 0 value

    Wire.begin();   // Init I2C for MPU6050
    TWBR = 24;      // 400kHz I2C clock

    Serial.begin(115200); // Debug
    Serial1.begin(9600);  // Bluetooth

    // initialize MPU
    if(debug) Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection to MPU
    if(debug) Serial.println(F("Testing device connections..."));
    if(debug) Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    if(debug) Serial.println(F("Initializing DMP..."));
    delay(500); // do not remove, will crash
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    //Your offsets:   6231    -3209   10806   169     88      39
    //Data is printed as: acelX acelY acelZ giroX giroY giroZ
    mpu.setXAccelOffset(6231);
    mpu.setYAccelOffset(-3209);
    mpu.setZAccelOffset(10806);
    mpu.setXGyroOffset(169);
    mpu.setYGyroOffset(88);
    mpu.setZGyroOffset(39);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) 
    {
        // turn on the DMP, now that it's ready
        if(debug) Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        if(debug) Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        if(debug) Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } 
    else 
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        if(debug) Serial.print(F("DMP Initialization failed (code "));
        if(debug) Serial.print(devStatus);
        if(debug) Serial.println(F(")"));
    }

    // Init PID variables
    pitchOutput = 0.0;
    rollOutput = 0.0;
    setPitch = 0.0;
    setRoll = 0.0;
    pitchInput = 0.0;
    rollInput = 0.0;
    
    // Configure PID
    pitchPID.SetMode(AUTOMATIC);
    rollPID.SetMode(AUTOMATIC);   
    
    pitchPID.SetOutputLimits(-100, 100);
    rollPID.SetOutputLimits(-100, 100);

    pinMode(DEBUG_PIN, OUTPUT);
    digitalWrite(DEBUG_PIN, 0);
    
    // pinMode(8, OUTPUT);
    // digitalWrite(8, 0);
    // pinMode(9, OUTPUT);
    // digitalWrite(9, 0);
    // pinMode(10, OUTPUT);
    // digitalWrite(10, 0);
    // pinMode(11, OUTPUT);
    // digitalWrite(11, 0);

    mpu.setIntDataReadyEnabled(1);

    Serial.print("getIntDataReadyEnabled: ");
    Serial.println(mpu.getIntDataReadyEnabled());
}

unsigned long micros_time = 0;
unsigned long micros_time_base = 0;

void Generate_Motor_Pulses()
{
    digitalWrite(8, HIGH);
    digitalWrite(9, HIGH);
    digitalWrite(10, HIGH);
    digitalWrite(11, HIGH);

    micros_time_base = micros();

    while(micros_time < micros_time_base + 2000)
    {
        if(micros_time > micros_time_base + motor_upper_left_us)
            digitalWrite(8, LOW);
        if(micros_time > micros_time_base + motor_upper_right_us)
            digitalWrite(9, LOW);
        if(micros_time > micros_time_base + motor_lower_left_us)
            digitalWrite(10, LOW);
        if(micros_time > micros_time_base + motor_lower_right_us)
            digitalWrite(11, LOW);

        micros_time = micros();
        if(debug) Serial.print("micros = ");
        if(debug) Serial.println(micros_time);
    }
}

// void setValues(double p1, double p2, double p3, double p4) // range [0; 100]
// {
//     motor_upper_left_us = calcMicroseconds(p1);
//     motor_upper_right_us = calcMicroseconds(p2);
//     motor_lower_left_us = calcMicroseconds(p3);
//     motor_lower_right_us = calcMicroseconds(p4);
// }

void Update_Motor_Values()
{// Read MPU data
        digitalWrite(DEBUG_PIN, 1);
        
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        mpu.dmpGetQuaternion(&quater, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &quater);
        mpu.dmpGetYawPitchRoll(ypr, &quater, &gravity);
        pitchInput = (ypr[1] * 180/M_PI);
        rollInput = (ypr[2] * 180/M_PI);

        if(debug) Serial.print("PR:");
        if(debug) Serial.print(pitchInput);
        if(debug) Serial.print(",");
        if(debug) Serial.println(rollInput);
       

        //Update Pids
        pitchPID.Compute();
        rollPID.Compute();

        // Apply PID output to motor PWM
        double motor_upper_left = speedRequest + (rollOutput / 2) + (pitchOutput / 2);
        double motor_upper_right = speedRequest - (rollOutput / 2) + (pitchOutput / 2);
        double motor_lower_left = speedRequest + (rollOutput / 2) - (pitchOutput / 2);
        double motor_lower_right = speedRequest - (rollOutput / 2) - (pitchOutput / 2);

        if(motor_upper_left < 0) motor_upper_left = 0;
        if(motor_upper_right < 0) motor_upper_right = 0;
        if(motor_lower_left < 0) motor_lower_left = 0;
        if(motor_lower_right < 0) motor_lower_right = 0;

        // if(debug) Serial.print("setValues(");
        // if(debug) Serial.print(motor_upper_left);
        // if(debug) Serial.print(",");
        // if(debug) Serial.print(motor_upper_right);
        // if(debug) Serial.print(",");
        // if(debug) Serial.print(motor_lower_left);
        // if(debug) Serial.print(",");
        // if(debug) Serial.print(motor_lower_right);
        // if(debug) Serial.println(")");
        
        // Write motor PWM
        setValues(motor_upper_left, motor_upper_right, motor_lower_left, motor_lower_right);
        mpuInterrupt = false;
        
        digitalWrite(DEBUG_PIN, 0);
}

void loop() 
{
    // if(debug) Serial.println(F("if (!dmpReady)"));
    
    if (!dmpReady)  return;

    // if(debug) Serial.println(F("readRcControllerData"));

    // Get command from Bluetooth
    readRcControllerData();

    bool isAllowedEngineUpdateByPid = true;

    if (isShutDownRequested())
    {
        isAllowedEngineUpdateByPid = false;
        shutDownQ();
    }

    if (!isMpuDataActual())
    {
        isAllowedEngineUpdateByPid = false;
        shutDownQ();
        clearControlRequestValues();
    }

    if(mpuInterrupt)
        Update_Motor_Values();

    // if(debug) Serial.print(F("while (!mpuInterrupt && fifoCount < packetSize)"));
    // if(debug) Serial.print(mpuInterrupt);
    // if(debug) Serial.println(fifoCount);
    
    
}

void clearControlRequestValues()
{
    readyToFlyStatus = false;
    speedRequest = 0;
}

void shutDownQ()
{
    setValues(0, 0, 0, 0);
}

bool isMpuDataActual()
{
    if (millis() - lastMpuReaded > MaxTimeWaitingForMpuData)
        return false;
    else
        return true;
}

bool isShutDownRequested()
{
    if (!rcCheckStatus)
        return true;

    if (!readyToFlyStatus)
        return true;

    if (speedRequest <= 0)
        return true;

    return false;
}

void waitData(int data)
{
    while (true)
        if(Serial1.available() > 0)
            if (Serial1.read() == data)
                break;
}

void readRcControllerData()
{
    if (!rcCheckStatus)
    {
        if(debug) Serial.println(F("For Calibration send 'k', to skip anysing else"));
        while (true)
        {
            if(Serial1.available() > 0)
            {
                if (Serial1.read() == 'k')
                {
                    if(debug) Serial.println(F("Disconnect engine power, then send 'k'"));
                    waitData('k');
                    setValues(100, 100, 100, 100);
                    if(debug) Serial.println(F("Connect engine power, wait for signal, then send 'k'"));
                    waitData('k');
                    setValues(0, 0, 0, 0);
                    if(debug) Serial.println(F("Calibration ended, send 'k'"));
                    waitData('k');
                }
            break;
            }
        }

        if(debug) Serial.println(F("Wait 8 then 9 symbols to init RC controller"));
        waitData('8');
        waitData('9');
        rcCheckStatus = true;
        if(debug) Serial.println(F("RC controller checked"));
    }

    //TODO: update to use sinle custom check data protocol
    if(Serial1.available() <= 0)
        return;

    int byte = Serial1.read();

    if (!readyToFlyStatus) 
    {        
        if (byte == '1')
        {
            readyToFlyStatus = true;
        }
        speedRequest = 0;
        return;
    }

    switch(byte)
    {
         case '+':
            if (speedRequest < 100)
                ++speedRequest;
            break;
         case '-':
            if (speedRequest > 0)
                --speedRequest;
            break;
         case '0':
            speedRequest = 0;
            readyToFlyStatus = false;
            break;
    }

    while (Serial.available() && Serial.read()); // empty buffer
}
