#include <stdlib.h>  
#include "I2Cdev.h"
#include <SoftwareSerial.h>  

#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

#define OUTPUT_PICARD
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno
#define LED_PIN 13 // Arduino is 13
bool blinkState = false;

// MPU control / status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };


// indicates whether MPU interrupt pin has gone high
volatile bool mpuInterrupt = false;     
void dmpDataReady() {
    mpuInterrupt = true;
}

// ================================================================
// =                        INITIAL SETUP                         =
// ================================================================
int bluetoothTx = 4;  // TX-O pin of bluetooth mate, Arduino D4
int bluetoothRx = 5;  // RX-I pin of bluetooth mate, Arduino D5

#define FADE_OUT_SPEED 0.01

// Random seed output
int seed = 10; 
// Changes in rotational velocity
float angleDeltas = 0; 
// Detection of shake gesture
float shakeVal = 0; 
// Saves previous recorded angles
float euler_prev[] = {0, 0, 0};

SoftwareSerial bluetooth(bluetoothTx, bluetoothRx);

#define SERIAL_OUT Serial

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    
    SERIAL_OUT.begin(115200); // initialize serial communication
    bluetooth.begin(115200); // Start bluetooth serial at 115200 baud

    // initialize device
    SERIAL_OUT.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    SERIAL_OUT.println(F("Testing device connections..."));
    SERIAL_OUT.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    SERIAL_OUT.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // Gyro offsets, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        SERIAL_OUT.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        SERIAL_OUT.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        SERIAL_OUT.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        SERIAL_OUT.print(F("DMP Initialization failed (code "));
        SERIAL_OUT.print(devStatus);
        SERIAL_OUT.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}


// ================================================================
// =                      MAIN PROGRAM LOOP                       =
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {}

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        SERIAL_OUT.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        fifoCount -= packetSize;

        #ifdef OUTPUT_PICARD
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

            float currDelta;
            for (int i = 0; i < 3; i++) {
              currDelta = fabs(euler[i] - euler_prev[i]);
              if (currDelta > M_PI) {
                currDelta = fabs(currDelta - 2*M_PI);
              }
              angleDeltas += currDelta*currDelta*5;
              
              euler_prev[i] = euler[i];
            }

            if (abs(aaReal.x) + abs(aaReal.y) + abs(aaReal.z) > 15000) {
              shakeVal = 1;
              angleDeltas = 1;
              seed += 1;
            }
            
            shakeVal -= FADE_OUT_SPEED;
            angleDeltas -= FADE_OUT_SPEED;
            
            if (angleDeltas < 0)
              angleDeltas = 0;
            if (angleDeltas > 1)
              angleDeltas = 1;
       
            if (shakeVal < 0)
              shakeVal = 0;
            if (shakeVal > 1)
              shakeVal = 1;

            SERIAL_OUT.print(euler[0]);
            SERIAL_OUT.print("\t");
            SERIAL_OUT.print(euler[1]);
            SERIAL_OUT.print("\t");
            SERIAL_OUT.print(euler[2]);
            SERIAL_OUT.print("\t");
            SERIAL_OUT.print(shakeVal);
            SERIAL_OUT.print("\t");
            SERIAL_OUT.print(angleDeltas);
            SERIAL_OUT.print("\t");
            SERIAL_OUT.println(seed);

        #endif

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}
