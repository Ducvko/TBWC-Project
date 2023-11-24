#include "Arduino.h"
#include "OffsetConfig.h"


// Constructor assigns the private variable sensor__ with the passed in MPU6050 sensor
OffsetConfig::OffsetConfig(MPU6050 *sensor) {
  sensor__ = sensor;
}

// The main routine that calibrates the offsets for the MPU6050 sensor
bool OffsetConfig::startCalibration() {
  Serial.println("\nReading Values for the First Time...");
  meansensors__(); // takes the average of the first 101 values read from the sensor to smooth out the sensor readings
  delay(1000);

  bool CalibResult;

  Serial.println("\nCalculating offsets...");
  CalibResult=calibration__(); // calibrates the offsets of the MPU 6050 sensor in this function
  if (CalibResult) {
    Serial.println("\nCalibration successful!");
  }
  else {
    Serial.println("\nCalibration failed!");
    while(1);
  }
  delay(1000);
}

void OffsetConfig::meansensors__() {
  long i=0,buff_ax=0,buff_ay=0,buff_az=0,buff_gx=0,buff_gy=0,buff_gz=0;
  
  while (i <(buffersize+discardfirstmeas+1)){
    // read raw accel/gyro measurements from sensor
    sensor__->getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    if (i>discardfirstmeas && i<=(buffersize+discardfirstmeas)){ //First 100 measures are discarded
      buff_ax=buff_ax+ax;
      buff_ay=buff_ay+ay;
      buff_az=buff_az+az;
      buff_gx=buff_gx+gx;
      buff_gy=buff_gy+gy;
      buff_gz=buff_gz+gz;
    }
    if (i==(buffersize+100)){
      mean_ax=buff_ax/buffersize;
      mean_ay=buff_ay/buffersize;
      mean_az=buff_az/buffersize;
      mean_gx=buff_gx/buffersize;
      mean_gy=buff_gy/buffersize;
      mean_gz=buff_gz/buffersize;
    }
    i++;
    delay(2); //Needed so we don't get repeated measures
  }
  
  Serial.print("Results of measurements a/g:\t");
  Serial.print(mean_ax); Serial.print("\t");
  Serial.print(mean_ay); Serial.print("\t");
  Serial.print(mean_az); Serial.print("\t");
  Serial.print(mean_gx); Serial.print("\t");
  Serial.print(mean_gy); Serial.print("\t");
  Serial.println(mean_gz);
}

/*
 * This Function will take the average results of the sensor readings and will try to
 * generate offset values so that the sensor readings converge to within 
 * the threshold specified in the header file
 * 
 * Will loop until all offsets result in a value that is within the 
 * threshold or until 100 loops occur resulting in a failed calibration
 */
bool OffsetConfig::calibration__() {
  int loopcount = 0;
  ax_offset=-mean_ax/accel_offset_divisor;
  ay_offset=-mean_ay/accel_offset_divisor;
  az_offset=(16384-mean_az)/8;

  gx_offset=-mean_gx/gyro_offset_divisor;
  gy_offset=-mean_gy/gyro_offset_divisor;
  gz_offset=-mean_gz/gyro_offset_divisor;

  while (1){
    int ready=0;
    sensor__->setXAccelOffset(ax_offset+ax_initoffset);
    sensor__->setYAccelOffset(ay_offset+ay_initoffset);
    sensor__->setZAccelOffset(az_offset+az_initoffset);

    sensor__->setXGyroOffset(gx_offset+gx_initoffset);
    sensor__->setYGyroOffset(gy_offset+gy_initoffset);
    sensor__->setZGyroOffset(gz_offset+gz_initoffset);

    meansensors__();
    Serial.println("...");

    if (abs(mean_ax)<=acel_deadzone) ready++;
    else ax_offset=ax_offset-mean_ax/acel_deadzone;

    if (abs(mean_ay)<=acel_deadzone) ready++;
    else ay_offset=ay_offset-mean_ay/acel_deadzone;

    if (abs(16384-mean_az)<=acel_deadzone) ready++;
    else az_offset=az_offset+(16384-mean_az)/acel_deadzone;

    if (abs(mean_gx)<=gyro_deadzone) ready++;
    else gx_offset=gx_offset-mean_gx/(gyro_deadzone+1);

    if (abs(mean_gy)<=gyro_deadzone) ready++;
    else gy_offset=gy_offset-mean_gy/(gyro_deadzone+1);

    if (abs(mean_gz)<=gyro_deadzone) ready++;
    else gz_offset=gz_offset-mean_gz/(gyro_deadzone+1);

    Serial.print("Resulting offset calibration value a/g:\t");
    Serial.print(ax_offset+ax_initoffset); Serial.print("\t");
    Serial.print(ay_offset+ay_initoffset); Serial.print("\t");
    Serial.print(az_offset+az_initoffset); Serial.print("\t");
    Serial.print(gx_offset+gx_initoffset); Serial.print("\t");
    Serial.print(gy_offset+gy_initoffset); Serial.print("\t");
    Serial.println(gz_offset+gz_initoffset);

    loopcount=loopcount+1;
    Serial.print("Loop Cnt: ");Serial.println(loopcount);
    if (loopcount==100) {
      return false;   
      break; // exit the calibration routine if no stable results can be obtained after 20 calibration loops
    }

    if (ready==6) {
     return true;   
     break;
    }

  }
}