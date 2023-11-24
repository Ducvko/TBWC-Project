#ifndef OffsetConfig_h
#define OffsetConfig_h

#include "Arduino.h"
#include "MPU6050_6Axis_MotionApps612.h"
#include <helper_3dmath.h>

class OffsetConfig {
  public:
    OffsetConfig(MPU6050 *sensor);
    bool startCalibration();
  private:

    void meansensors__();
    bool calibration__();

    MPU6050 *sensor__;
    int16_t ax,ay,az,gx,gy,gz;
    int mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz,state=0;
    int ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset;
    int ax_initoffset = 0,ay_initoffset = 0,az_initoffset = 0,gx_initoffset = 0,gy_initoffset = 0,gz_initoffset = 0;
    int buffersize=1000;     //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
    int discardfirstmeas=100;  // Amount of initial measurements to be discarded
    int acel_deadzone=7;     //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
    int gyro_deadzone=0.09;     //Gyro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)
    int accel_offset_divisor=8; //8;
    int gyro_offset_divisor=4; //4;
    
};

#endif