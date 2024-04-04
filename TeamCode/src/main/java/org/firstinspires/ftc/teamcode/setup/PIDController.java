package org.firstinspires.ftc.teamcode.setup;

public class PIDController {
    private double kp = 0.1; // Proportional gain
    private double ki = 0.01; // Integral gain
    private double kd = 0.1; // Derivative gain

    private double previousError;
    private double integral;
    private double previousTime;

    public PIDController() {
        this.previousTime = System.currentTimeMillis() / 1000.0;
    }

    public double calculate(double setpoint, double measurement) {
        double currentTime = System.currentTimeMillis() / 1000.0;
        double dt = currentTime - previousTime;

        double error = setpoint - measurement;
        integral += error * dt;
        double derivative = (error - previousError) / dt;

        double output = kp * error + ki * integral + kd * derivative;

        previousError = error;
        previousTime = currentTime;

        return output;
    }
}