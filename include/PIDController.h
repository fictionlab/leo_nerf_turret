#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

class PIDController {
private:
    double kp;  // Proportional gain
    double ki;  // Integral gain
    double kd;  // Derivative gain
    double prevError;  // Previous error value

    double integral;  // Integral sum
    
    double value_cap; // max value to return

public:
    PIDController(double kp, double ki, double kd, double value_cap);
    double calculate(double setpoint, double processVariable, double dt);
};

#endif  // PID_CONTROLLER_HPP
