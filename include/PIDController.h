#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

class PIDController {
private:
    float kp;  // Proportional gain
    float ki;  // Integral gain
    float kd;  // Derivative gain
    float prevError;  // Previous error value

    float integral;  // Integral sum
    
    float value_cap; // max value to return

public:
    PIDController(float kp, float ki, float kd, float value_cap);
    float calculate(float setpoint, float processVariable, float dt);

    void tuneKp(float kp);
    void tuneKi(float ki);
    void tuneKd(float kd);
    void tune(float kp, float ki, float di);
};

#endif  // PID_CONTROLLER_HPP
