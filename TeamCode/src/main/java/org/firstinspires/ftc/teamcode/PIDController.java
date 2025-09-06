package org.firstinspires.ftc.teamcode;

public class PIDController {
    private double kP, kI, kD;
    private double setpoint = 0;
    private double integral = 0;
    private double previousError = 0;
    private double outputMin = Double.NEGATIVE_INFINITY;
    private double outputMax = Double.POSITIVE_INFINITY;

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    // Set target
    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    // Optional: set output limits
    public void setOutputLimits(double min, double max) {
        this.outputMin = min;
        this.outputMax = max;
    }

    // Reset integral and previous error (useful when starting a new motion)
    public void reset() {
        integral = 0;
        previousError = 0;
    }

    // Calculate output for a given input and deltaTime
    public double calculate(double input, double deltaTime) {
        double error = setpoint - input;
        integral += error * deltaTime;
        double derivative = (error - previousError) / deltaTime;

        double output = (kP * error) + (kI * integral) + (kD * derivative);

        // Clamp output
        if (output > outputMax) output = outputMax;
        if (output < outputMin) output = outputMin;

        previousError = error;
        return output;
    }
}
