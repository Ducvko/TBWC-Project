#include "Arduino.h"
#include "DifferentialKinematicsModel.h"


/*
 * Assigns the private values _trackWidth, _linearVel, and _angularVel to the width between the back wheels in meters, 
 * the linear velocity in m/s and the angular velocity in rad/s respectively
 */
DifferentialKinematicsModel::DifferentialKinematicsModel(float trackWidth, float *linearVel, float *angularVel) {
    _trackWidth = trackWidth;
    _linearVel = linearVel;
    _angularVel = angularVel;
}

/*
 * A derivation of the differential drivetrain equation which translates the linear velocity and angular velocity of a chassis
 * to the individual wheel speeds of the drivetrain which will cause the identical linear velocity and angular velocity onto
 * the chassis
 */
void DifferentialKinematicsModel::calculate() {
    lWheelSpeed = constrain(*_linearVel + _trackWidth / 2 * *_angularVel, -255, 255);
    rWheelSpeed = constrain(*_linearVel - _trackWidth / 2 * *_angularVel, -255, 255);
}