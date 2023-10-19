#include <DifferentialKinematicsModel.h>
#include <OffsetConfig.h>
#include <PIDController.h>

#include <helper_3dmath.h>


#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define LED_PIN 13 // debug pin

#define ENA 0     // TODO: Determine Pins Being Used
#define IN1A 0    // TODO: Determine Pins Being Used -- PWM
#define IN2A 0    // TODO: Determine Pins Being Used -- PWM

#define ENB 0     // TODO: Determine Pins Being Used
#define IN1B 0    // TODO: Determine Pins Being Used -- PWM
#define IN2B 0    // TODO: Determine Pins Being Used -- PWM


/*-------------------------DMP Config Constants---------------------------*/

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

/*---------------------------Data Containers------------------------------*/

Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements; LSB Units
VectorFloat gravity;    // [x, y, z]            gravity vector
VectorInt16 angularVelVector; // [x, y, z]            Angular Velocity measurements; Deg/s Units
float angularVel;

float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

float yaw;              // Degrees
float pitch;            // Degrees
float roll;             // Degrees

float linAccel_x;       // m*s^-2
float linAccel_y;       // m*s^-2
float linAccel_z;       // m*s^-2

float velocity;
float vel_x;            // m*s^-1
float vel_y;            // m*s^-1
float vel_z;            // m*s^-1

float velPIDOutput;
float yawPIDOutput;

/*-----------------------------Misc. Variables----------------------------*/

int prevTime;
int currentTime;
int time_for_collection;
int deltaTime;

float trackWidth = 20; // Units and value to be measured

/*----------------------Custom Class Instantiations-----------------------*/

MPU6050 mpu; // MPU 6050 Declaration
OffsetConfig configOffsetObj(&mpu); // Custom object for configuring the axis offset of the MPU6050
PIDController velPIDController(&velPIDOutput, &velocity);
PIDController yawPIDController(&yawPIDOutput, &angularVel);
DifferentialKinematicsModel differentialDrive(trackWidth, &velPIDOutput, &yawPIDOutput);

/*--------------------------PID Control Variables-------------------------*/

float vel_kP = 0;     // TODO Tune Velocity PID Constants
float vel_kI = 0;     // TODO Tune Velocity PID Constants
float vel_kD = 0;     // TODO Tune Velocity PID Constants

float yaw_kP = 0;     // TODO: Tune Angular Velocity PID Constants
float yaw_kI = 0;     // TODO: Tune Angular Velocity PID Constants
float yaw_kD = 0;     // TODO: Tune Angular Velocity PID Constants

float yawPlant = 0.0; // TODO: Tune straight offset

float velPlant; // TODO Find optimal chassis speed


/*========================================================================*/
/*------------------------Custom Method Declarations----------------------*/
/*========================================================================*/


void MPUConfig() {
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  Serial.begin(9600);
  while(!Serial);

  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again

  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);

  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);

  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

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
}

void readFIFOBuffer() {
  mpu.resetFIFO();

  fifoCount = mpu.getFIFOCount();

  while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

  mpu.getFIFOBytes(fifoBuffer, packetSize);
  
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetAccel(&aa, fifoBuffer);
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  mpu.dmpGetGyro(&angularVelVector, fifoBuffer);


  linAccel_x = aaReal.x / (2 * (float) 16384) * 19.6; // Convert from DMP range of +- 2g (+- 16384) to m/s^2
  linAccel_y = aaReal.y / (2 * (float) 16384) * 19.6; // Convert from DMP range of +- 2g (+- 16384) to m/s^2
  linAccel_z = aaReal.z / (2 * (float) 16384) * 19.6; // Convert from DMP range of +- 2g (+- 16384) to m/s^2

  yaw = ypr[0];
  pitch = ypr[1];
  roll = ypr[2];
}

void spinMotors(DifferentialKinematicsModel differentialModel) {

  if (differentialModel.lWheelSpeed > 0) {
    analogWrite(IN1A, differentialModel.lWheelSpeed);
    digitalWrite(IN2A, LOW);
  } else {
    analogWrite(IN2A, differentialModel.lWheelSpeed);
    digitalWrite(IN1A, LOW);
  }

  if (differentialModel.rWheelSpeed > 0) {
    analogWrite(IN2B, differentialModel.rWheelSpeed);
    digitalWrite(IN1B, LOW);
  } else {
    analogWrite(IN1B, differentialModel.rWheelSpeed);
    digitalWrite(IN2B, LOW);
  }

}

/*=======================================================================*/
/*--------------------------Main Setup and Loop--------------------------*/
/*=======================================================================*/

void setup() {

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  digitalWrite(ENA, HIGH);
  digitalWrite(ENB, HIGH);

  pinMode(IN1A, OUTPUT);
  pinMode(IN2A, OUTPUT);
  pinMode(IN1B, OUTPUT);
  pinMode(IN2B, OUTPUT);

  MPUConfig();

  configOffsetObj.startCalibration();

  velPIDController.setPID(vel_kP, vel_kI, vel_kD);

  yawPIDController.setPID(yaw_kP, yaw_kI, yaw_kD);

  yawPIDController.setPlant(0);


  prevTime = millis();
}

void loop() {
  
  if (!dmpReady) return;

  currentTime = millis();

  readFIFOBuffer();

  time_for_collection = currentTime - millis();

  deltaTime = currentTime - prevTime - time_for_collection;

  // TODO: Find which axis is front facing on physical model
  vel_x = aaReal.x * (deltaTime / (float) 1000);
  vel_y = aaReal.y * (deltaTime / (float) 1000);
  vel_z = aaReal.z * (deltaTime / (float) 1000);  

  velPIDController.calculate(time_for_collection / (float) 1000); 

  yawPIDController.calculate(time_for_collection / (float) 1000);

  differentialDrive.calculate();

  spinMotors(differentialDrive);

  prevTime = currentTime;
}
