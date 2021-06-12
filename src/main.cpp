#include <Arduino.h>

#include "I2Cdev.h"
#include <PID_v1.h>

#include "MPU6050_6Axis_MotionApps20.h"

#include "motordriver.h"

MPU6050 mpu;

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
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
void dmpDataReady() 
{
    mpuInterrupt = true;
}

uint64_t lastMpuReaded = 0;
const uint64_t MaxTimeWaitingForMpuData = 100;

void readRcControllerData();
bool isShutDownRequested();
bool isMpuDataActual();
void shutDownQ();
void clearControlRequestValues();

void setup() 
{
    setupDriver();

    Wire.begin();
    TWBR = 24; // 400kHz I2C clock

    Serial.begin(115200); // Debug
    Serial1.begin(9600);  // BLT


// byte count = 0;
// Wire.begin();
// for (byte i = 8; i < 120; i++)
// { Wire.beginTransmission (i);
// if (Wire.endTransmission () == 0)
// {
// Serial.print ("Found address: ");
// Serial.print (i, DEC);
// Serial.print (" (0x");
// Serial.print (i, HEX);
// Serial.println (")");
// count++;
// delay (1);
// } // end of good response
// } // end of for loop

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    // Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    delay(500);
    // while (Serial.available() && Serial.read()); // empty buffer
    // while (!Serial.available());                 // wait for data
    // while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
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
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure stabilization code
    //=============================
    pitchOutput = 0.0;
    rollOutput = 0.0;
    setPitch = 0.0;
    setRoll = 0.0;
    pitchInput = 0.0;
    rollInput = 0.0;
    
    pitchPID.SetMode(AUTOMATIC);
    rollPID.SetMode(AUTOMATIC);   
    
    pitchPID.SetOutputLimits(-100, 100);
    rollPID.SetOutputLimits(-100, 100);

    pinMode(13, OUTPUT);
    pinMode(12, OUTPUT);

    }

void loop() 
{
    digitalWrite(13, blinkState);
    blinkState = !blinkState;

    // Serial.println(F("if (!dmpReady)"));
    if (!dmpReady) 
        return;

    // Serial.println(F("readRcControllerData"));

    // Get data from Bluetooth
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

    // Serial.print(F("while (!mpuInterrupt && fifoCount < packetSize)"));
    // Serial.print(mpuInterrupt);
    // Serial.println(fifoCount);
    
    while (!mpuInterrupt && fifoCount < packetSize) 
    {
        static bool st = false;
        st = !st;
        digitalWrite(12, st);

        if (!isMpuDataActual())
        {
            shutDownQ();
            clearControlRequestValues();
            return;
        }

        if (!isAllowedEngineUpdateByPid)
            continue;

        double motor1_upper_left = speedRequest + (rollOutput / 2) + (pitchOutput / 2);
        double motor2_upper_right = speedRequest - (rollOutput / 2) + (pitchOutput / 2);
        double motor3_lower_left = speedRequest + (rollOutput / 2) - (pitchOutput / 2);
        double motor4_lower_right = speedRequest - (rollOutput / 2) - (pitchOutput / 2);

        // Serial.println(F("setValues(p1, p2, p3, p4);"));

        setValues(motor1_upper_left, motor2_upper_right, motor3_lower_left, motor4_lower_right);
    }

    mpuInterrupt = false;

    // Serial.println(F("mpu.getIntStatus"));
    mpuIntStatus = mpu.getIntStatus();

    // Serial.println(F("mpu.getFIFOCount"));
    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & 0x10) || fifoCount == 1024) 
    {
        // Serial.println(F("mpu.resetFIFO();"));
        mpu.resetFIFO();
        // Serial.println(F("FIFO overflow!"));
    } 
    else 
    {
        // Serial.println(F("if ((mpuIntStatus & 0x02) > 0)"));
        if ((mpuIntStatus & 0x02) > 0) 
        {
            // Serial.println(F("while (fifoCount < packetSize) "));
            
            constexpr uint32_t timeout = 100;
            uint32_t t1 = millis();
            while (fifoCount < packetSize)
            {
                fifoCount = mpu.getFIFOCount();
                if (millis() - t1 > timeout)
                    break;
            }
            
            // Serial.println(F("mpu.getFIFOBytes(fifoBuffer, packetSize);"));
            lastMpuReaded = millis();

            mpu.getFIFOBytes(fifoBuffer, packetSize);
        
            fifoCount -= packetSize;

            mpu.dmpGetQuaternion(&quater, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &quater);
            mpu.dmpGetYawPitchRoll(ypr, &quater, &gravity);
            pitchInput = (ypr[1] * 180/M_PI);
            rollInput = (ypr[2] * 180/M_PI);

            //Update Pids
            pitchPID.Compute();
            rollPID.Compute();

            #ifdef DEBUG_OUTPUT
                Serial.print("gravity (x, y, z): ");
                Serial.print(gravity.x);
                Serial.print(" ");
                Serial.print(gravity.y);
                Serial.print(" ");
                Serial.print(gravity.z);

                Serial.print("   (pitchIn, rollIn): ");
                Serial.print(pitchInput);
                Serial.print(" ");
                Serial.print(rollInput);

                Serial.print("   (pitchOut, rollOut): ");
                Serial.print(pitchOutput);
                Serial.print(" ");
                Serial.print(rollOutput);
            #endif
        }
    // Serial.println(F("end"));
    }
// Serial.println(F("END"));
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
        Serial.println(F("For Calibration send 'k', to skip anysing else"));
        while (true)
        {
            if(Serial1.available() > 0)
            {
                if (Serial1.read() == 'k')
                {
                    Serial.println(F("Disconnect engine power, then send 'k'"));
                    waitData('k');
                    setValues(100, 100, 100, 100);
                    Serial.println(F("Connect engine power, wait for signal, then send 'k'"));
                    waitData('k');
                    setValues(0, 0, 0, 0);
                    Serial.println(F("Calibration ended, send 'k'"));
                    waitData('k');
                }
            break;
            }
        }

        Serial.println(F("Wait 8 then 9 symbols to init RC controller"));
        waitData('8');
        waitData('9');
        rcCheckStatus = true;
        Serial.println(F("RC controller checked"));
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
