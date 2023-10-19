#ifndef PIDController_h
#define PIDController_h

#include "Arduino.h"

class PIDController {

  public:
    PIDController(float *output, float *sensorData);
    void setPID(float kP, float kI, float kD);
    void setPlant(float setPoint);
    void calculate(float collectionTime);
  private:
    float *_output;
    float *_sensorData;

    float _prevTimeMillis = millis();
    float _currentTime;
    float _deltaTime;

    float _errorSum;
    float _error;
    float _previousError = 0.001;
    float _deltaError;
    void _calculateError();

    float _kP;
    float _kI;
    float _kD;
    float _calculateP();
    float _calculateI();
    float _calculateD();

    float _setpoint;

};

#endif