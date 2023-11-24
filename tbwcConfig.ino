#include <DifferentialKinematicsModel.h>
#include <OffsetConfig.h>
#include <PIDController.h>

#include <helper_3dmath.h>


#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define LED_PIN 13 // debug pin

#define ENA 4     
#define IN1A 6    
#define IN2A 5     

#define ENB 8     
#define IN1B 10    
#define IN2B 9    

#define IR 2

#define dribble 0

/*-------------------------DMP Config Constants---------------------------*/

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

float imu_period = 0.01;

/*---------------------------Data Containers------------------------------*/

Quaternion q;                   // [w, x, y, z]         quaternion container
VectorInt16 aa;                 // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;             // [x, y, z]            gravity-free accel sensor measurements; LSB Units
VectorFloat gravity;            // [x, y, z]            gravity vector

float ypr[3];                   // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float yaw = 0;                      // Degrees
float pitch;                    // Degrees
float roll;                     // Degrees

VectorInt16 angularVelVector;   // [x, y, z]            Angular Velocity measurements; Deg/s Units
float angularVel;

float linAccel_x;               // m*s^-2
float linAccel_y;               // m*s^-2
float linAccel_z;               // m*s^-2

float velocity = 0;
float vel_x;                    // m*s^-1
float vel_y;                    // m*s^-1
float vel_z;                    // m*s^-1


/*--------------------------PID Control Variables-------------------------*/

float yaw_kP = 2000;          // P Constant for YAW PID
float yaw_kI = 1200;          // I Constant for YAW PID
float yaw_kD = 15;            // D Constant for YAW PID

float velPIDOutput = 0.0F;    // The PID Output variable for VELOCITY
float yawPIDOutput = 0.0F;    // The PID Output variable for YAW

float yawPlant = 0.03F;       // PID Setpoint for YAW
float yaw_threshold = 0.01F;  // Value which the absolute YAW value must be bigger than for PID to continue operating

float throttle = 0.0F;

int count1 = 0; // tracker values for taking rolling averages to smooth out sensor data
int count2 = 0; // tracker values for taking rolling averages to smooth out sensor data
int size = 10;  // tracker values for taking rolling averages to smooth out sensor data
float accel_rolling_avg[10] = {0}; // tracker values for taking rolling averages to smooth out sensor data
float vel_rolling_avg[10] = {velocity}; // tracker values for taking rolling averages to smooth out sensor data
float yaw_rolling_avg[10] = {0}; // tracker values for taking rolling averages to smooth out sensor data

/*-----------------------------Misc. Variables----------------------------*/

int prev_time;
int current_time;

float trackWidth = 0.162; // Distance between wheels

bool startup = true;

bool enable = false;

bool switch_dir = false;

bool in_range = false;

/*----------------------Custom Class Instantiations-----------------------*/

MPU6050 mpu; // MPU 6050 Declaration
OffsetConfig configOffsetObj(&mpu); // Custom object for configuring the axis offset of the MPU6050
PIDController yawPIDController(&yawPIDOutput, &yaw, imu_period, PIDMode::POSITION); // custom PID object for YAW PID
DifferentialKinematicsModel differentialDrive(trackWidth, &throttle, &yawPIDOutput); // Custom Differential Drive object for drivetrain

/*========================================================================*/
/*------------------------Custom Method Declarations----------------------*/
/*========================================================================*/


/*
 * Configures The MPU6050 Sensor
*/
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

  // Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  // while (Serial.available() && Serial.read()); // empty buffer
  // while (!Serial.available());                 // wait for data
  // while (Serial.available() && Serial.read()); // empty buffer again

  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(80);
  mpu.setYGyroOffset(-58);
  mpu.setZGyroOffset(4);

  mpu.setXAccelOffset(-1482);
  mpu.setYAccelOffset(-2664);
  mpu.setZAccelOffset(1669);

  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

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

/*
 * Reads data from the MPU6050 FIFO Buffer and puts the values to their respecctive variables
 */
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


  float accel = (aaReal.x / (float) 16384) * 19.6; // Convert from DMP range of +- 2g (+- 16384) to m/s^2

  if (count1 < size) {
    accel_rolling_avg[count1] = accel;
    yaw_rolling_avg[count1] = ypr[0];
    count1++;
  } 

  float accel_sum = 0;
  float yaw_sum = 0;
  for (int i = 0; i < size; ++i) {
    accel_sum += accel_rolling_avg[i];
    yaw_sum += yaw_rolling_avg[i];
  }
  linAccel_x = (accel_sum / (float) size);
  float yaw_avg = (yaw_sum / (float) size);
  if (abs(yaw - yaw_avg) < 0.5) yaw = yaw_avg;

  if (count1 >= size) count1 = 0;
}

/*
 * Function to be called when the IR sensor tirggers when dribbling
 */
void switchDir() {
  // velPIDController.setPlant(-velPlant);
  throttle = -255;
  differentialDrive.calculate();
  spinMotors(differentialDrive);
  Serial.println("backwards!");
}

/*
 * Function to be called when IR sensor is Triggered when shooting
 */ 
void withinRange() {
  in_range = true;
}

/*
 * Function that wiggles the chassis to knock the moustrap trigger loose
 */
void fiddle() {
  throttle = 255;
  differentialDrive.calculate();
  spinMotors(differentialDrive);
}

/*
 * Function handles motor control through PWM and accounts for negative values
 */ 
void spinMotors(DifferentialKinematicsModel differentialModel) {

  if (differentialModel.lWheelSpeed > 0) {
    analogWrite(IN1A, differentialModel.lWheelSpeed);
    digitalWrite(IN2A, LOW);
  } else {
    analogWrite(IN2A, abs(differentialModel.lWheelSpeed));
    digitalWrite(IN1A, LOW);
  }

  if (differentialModel.rWheelSpeed > 0) {
    analogWrite(IN1B, differentialModel.rWheelSpeed);
    digitalWrite(IN2B, LOW);
  } else {
    analogWrite(IN2B, abs(differentialModel.rWheelSpeed));
    digitalWrite(IN1B, LOW);
  }
}

/*=======================================================================*/
/*--------------------------Main Setup and Loop--------------------------*/
/*=======================================================================*/

/*
 * Setup Function that Configures Pins and runs startup routines for PID controllers and the Gyro
 */
void setup() {

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(IN1A, OUTPUT);
  pinMode(IN2A, OUTPUT);
  pinMode(IN1B, OUTPUT);
  pinMode(IN2B, OUTPUT);

  digitalWrite(ENA, HIGH);
  digitalWrite(ENB, HIGH);

  pinMode(IR, INPUT_PULLUP);

  #if dribble == 1
    attachInterrupt(digitalPinToInterrupt(IR), switchDir, HIGH);
  #elif dribble == 0
    attachInterrupt(digitalPinToInterrupt(IR), withinRange, HIGH);
  #endif

  MPUConfig();
  
  enable = true;

  // configOffsetObj.startCalibration();

  yawPIDController.setPID(yaw_kP, yaw_kI, yaw_kD);

  yawPIDController.setThreshold(yaw_threshold);

  if (dmpReady) {
    for (int i = 0; i < 15; ++i) {
      readFIFOBuffer();
    }
    yawPlant = yaw;
    yawPIDController.setPlant(yawPlant);
  }

  #if dribble == 1
    throttle = 255;
  #elif dribble == 0
    throttle = 45;
  #endif

}


/* 
 * Main loop function that calculates PID outputs and main actions for the chassis to perform
 */
void loop() {
  current_time = millis();
  if (startup) prev_time = current_time - (float) (imu_period * 1000.0F), startup = false;
  
  if (enable) {

    if (!dmpReady) return;

    #if dribble == 1

      // Below calculates the PID output of the Yaw PID controller and sends it to the differential drivetrain which calculates the wheel speed from the output
      // Then sends the values to the motors

      readFIFOBuffer();

      yawPIDController.calculate(current_time - prev_time);

      differentialDrive.calculate();

      spinMotors(differentialDrive);

    #elif dribble == 0

      if (in_range) {
        // Below runs when the IR sensor is triggered and jerks the chassis to top speed then stops after 3 seconds
        fiddle();
        delay(3000);
        differentialDrive.lWheelSpeed = 0;
        differentialDrive.rWheelSpeed = 0;
        spinMotors(differentialDrive);
        while (true);
      } else {

        // Below drives the chassis at the slowest possible speed until the IR sensor is triggered

        readFIFOBuffer();

        // yawPIDController.calculate(current_time - prev_time);

        differentialDrive.calculate();
        Serial.println("------------------------");
        Serial.println("Yaw: \t\t" + String(yaw));
        Serial.println(differentialDrive.lWheelSpeed);
        Serial.println(differentialDrive.rWheelSpeed);

        spinMotors(differentialDrive);
      }

    #endif
  }
  prev_time = current_time;
}
