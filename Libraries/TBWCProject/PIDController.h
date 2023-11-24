#ifndef PIDController_h
#define PIDController_h

#include "Arduino.h"


typedef enum Mode{
  POSITION,
  VELOCITY
} PIDMode;

class PIDController {

  public:
    PIDController(float *output, float *sensorData, float period, PIDMode pidMode);
    void setPID(float kP, float kI, float kD);
    void setPlant(float setPoint);
    void setThreshold(float threshold);
    void calculate(int millis);
  private:
    PIDMode _pidMode;

    float *_sensorData;

    float _period = 0.01;

    bool _firstCycle = true;

    float _integral = 0;
    float _error = 0;
    float _previousError = _error;
    float _deltaError = 0;
    void _calculateError();

    float _setpoint;
    float _threshold;

    float _kP = 0;
    float _kI = 0;
    float _kD = 0;
    float _calculateP();
    float _calculateI();
    float _calculateD();
    float *_output;
};

#endif