#include "PIDController.h"

PIDController::PIDController(float kp, float ki, float kd,float value_cap) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->value_cap = value_cap;
    prevError = 0.0;
    integral = 0.0;
}

float PIDController::calculate(float setpoint, float processVariable, float dt) {
    float error = setpoint - processVariable;
    integral += error * dt;

    if(integral > value_cap){ integral = value_cap;}
    if(integral < -value_cap){ integral = -value_cap;}

    float derivative = (error - prevError) / dt;
    float output = kp * error + ki * integral + kd * derivative;
    prevError = error;

    if(output > value_cap){ output = value_cap;}
    if(output < -value_cap){ output = -value_cap;}

    return output;
}

void PIDController::tuneKp(float kp){
    this->kp = kp;
}
void PIDController::tuneKi(float ki){
    this->ki = ki;
}
void PIDController::tuneKd(float kd){
    this->kd = kd;
}
void PIDController::tune(float kp, float ki, float kd){
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}