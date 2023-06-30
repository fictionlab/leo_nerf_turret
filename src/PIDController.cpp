#include "PIDController.h"

PIDController::PIDController(double kp, double ki, double kd,double value_cap) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->value_cap = value_cap;
    prevError = 0.0;
    integral = 0.0;
}

double PIDController::calculate(double setpoint, double processVariable, double dt) {
    double error = setpoint - processVariable;
    integral += error * dt;

    if(integral > value_cap){ integral = value_cap;}
    if(integral < -value_cap){ integral = -value_cap;}

    double derivative = (error - prevError) / dt;
    double output = kp * error + ki * integral + kd * derivative;
    prevError = error;

    if(output > value_cap){ output = value_cap;}
    if(output < -value_cap){ output = -value_cap;}

    return output;
}