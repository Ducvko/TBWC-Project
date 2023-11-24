#include "Arduino.h"
#include "PIDController.h"

/*
 * The constructor sets the private variables: _output, _sensorData, PID Type, and _period to be
 * the Control Variable, Process Variable, Velocity or Positional PID Modes, and the polling period of the sensor respectively
 */
PIDController::PIDController(float *output, float *sensorData, float period, PIDMode mode) {
  _output = output;
  _sensorData = sensorData;
  _pidMode = mode;

  _period = period; // MPU 6050 Sampling time is 100 Hz or 0.01 s
}

/*
 * Sets the private PID constants to their respective values passed in as parameters.
 */
void PIDController::setPID(float kP, float kI, float kD) {
  _kP = kP;
  _kI = kI;
  _kD = kD;
}

/*
 * Sets the plant/setpoint of the PID Controller and resets the PID controller as if no previous
 * calculations were done previously
 */
void PIDController::setPlant(float setPoint) {
  _setpoint = setPoint;
  _integral = 0.0F;
  _firstCycle = true;
}

/*
 * Sets the Threshold value for the PID Controller
 * 
 * Only has an effect with Positional PID control
 */
void PIDController::setThreshold(float threshold) {
  _threshold = threshold;
}

/*
 * The main function of the PID Controller, Gets the accurate time between each call of this function as a parameter
 * and also calculates the current and change in error, then assigns the previous error to be the current error at the end of the function.
 * 
 * Depending on if it is a Positional PID controller the controller will not continue to calculate the output if the PV
 * is within the threshold, otherwise it will continue to calculate the output if it is outside the threshold or is a Velocity PID Controller.
 */
void PIDController::calculate(int millis) {
  _period = (float) millis / 1000.0F;
  _calculateError();

  if (_pidMode == PIDMode::POSITION) {
    if (abs(_error) > _threshold) {
      float _controlOut = _calculateP() + _calculateI() + _calculateD();
      *_output = _controlOut;
    } else {
      *_output = 0;
    }
  } else if (_pidMode == PIDMode::VELOCITY) {
    float _controlOut = _calculateP() + _calculateI() + _calculateD();
    *_output = _controlOut;
  }

  _previousError = _error;
}

/*
 * Calculates the current error and sets the previous error to the current error if this is the first calculation it is being called for.
 * Lastly it sets the change in error to the difference in the current and previous error.
 */
void PIDController::_calculateError() {
  _error = (_setpoint - *_sensorData);  
  if (_firstCycle) {
    _previousError = _error; 
    _firstCycle = false;
  }

  _deltaError = _error - _previousError;
}

/*
 * The first term in the PID equation: The Proportional Term, which is the product of the 
 * Proportional Constant or _kP as specified here and the current error in time
 */
float PIDController::_calculateP() {
  return _kP * _error;
}

/*
 * The Second Term in the PID Equation: The Integral Term, which is the sum of all errors multiplied by time
 * which is then multiplied by the Integral Constant or _kI as specified in the function.
 * 
 * There is an addition to the integral calculation in which it integrates the average between the current
 * error and the previous error with the change in time (the period), the result of this change is a 
 * rudementary version of something called Trapezoidal Motion Profiling which smooths out the change in the PV by the CV. 
 * To read more about Motion Profiling click here: https://www.ctrlaltftc.com/advanced/motion-profiling
 */
float PIDController::_calculateI() {
  _integral += (_period * (_error + _previousError) / 2); // trapeziod motion profiling, takes the average of the error then integrates it for smoothing; 
  return (_kI * _integral);
}

/*
 * The Third term in the PID Equation: The Derivitave Term, which is the rate of change in the error multiplied by the Derivitave constant
 * The purpose of this term is to counteract the overshoot and the overreactivity of the CV caused by the Integral term. 
 * 
 * The way it acheives this is best understood by a depiction of the PV's "position" over time, in an ideal PID controller
 * the PV will start shooting up to the setpoint. But as time passes the PV gets closer to the plant, and as such the error
 * decreases over time too. Because the error is decreasing the derivative of the error will be negative and will be counterracting 
 * the positive Proportional and Integral terms, causing a smooth ascension by the PV to the plant.
 */
float PIDController::_calculateD() {
  return (_kD * (_deltaError / _period));
}

