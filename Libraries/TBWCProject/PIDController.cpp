#include "Arduino.h"
#include "PIDController.h"


PIDController::PIDController(float *output, float *sensorData) {
    _output = output;
    _sensorData = sensorData;

    _prevTimeMillis = millis();
}

void PIDController::setPID(float kP, float kI, float kD) {
  _kP = kP;
  _kI = kI;
  _kD = kD;
}

void PIDController::setPlant(float setPoint) {
  _setpoint = setPoint;
}

void PIDController::calculate(float collectionTime) {
  _currentTime = millis();
  _deltaTime = (_currentTime - _prevTimeMillis - collectionTime) / (float) 1000;

  _calculateError();

  float _controlOut = _calculateP() + _calculateI() + _calculateD();

  *_output = _controlOut;

  _prevTimeMillis = _currentTime;
  _previousError = _error;
}

void PIDController::_calculateError() {
  _error = (_setpoint - *_sensorData);
  _errorSum += _error;
  _deltaError = _error - _previousError;
}

float PIDController::_calculateP() {
  return _kP * _error;
}

float PIDController::_calculateI() {
  return (_kI * _deltaTime * _errorSum);
}

float PIDController::_calculateD() {
  return (_kD / _deltaTime) * _deltaError;
}

