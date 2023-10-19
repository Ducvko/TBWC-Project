#include "Arduino.h"
#include "DifferentialKinematicsModel.h"

DifferentialKinematicsModel::DifferentialKinematicsModel(float trackWidth, float *linearVel, float *angularVel) {
    _trackWidth = trackWidth;
    _linearVel = linearVel;
    _angularVel = angularVel;
}

void DifferentialKinematicsModel::calculate() {
    lWheelSpeed = *_linearVel - _trackWidth / 2 * *_angularVel;
    rWheelSpeed = *_linearVel + _trackWidth / 2 * *_angularVel;
}