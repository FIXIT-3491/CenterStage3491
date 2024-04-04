package org.firstinspires.ftc.teamcode.setup;

public class PIDController {
    private double kp; // Proportional gain
    private double ki; // Integral gain
    private double kd; // Derivative gain

    private double previousError;
    private double integral;

    public PIDController(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }

    public double calculate(double setpoint, double measurement) {
        double error = setpoint - measurement;
        integral += error;
        double derivative = error - previousError;

        double output = kp * error + ki * integral + kd * derivative;

        previousError = error;

        return output;
    }
}