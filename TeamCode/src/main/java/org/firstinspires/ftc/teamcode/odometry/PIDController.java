package org.firstinspires.ftc.teamcode.odometry;

public class PIDController {

    double kP;
    double kI;
    double kD;
    double integralSum;
    double lastError;
    double target;
    ElapsedTime timer;
    boolean isFinished;

    final double ERROR_TOLERANCE = 0.01;
    
    
    public PIDController (double kP, double kI, double kD)
    {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.integralSum = 0;
        this.lastError = 0;
        this.target = 0;
        this.timer = new ElapsedTime();
        this.isFinished = false;
    }

    public void setTarget(double val)
    {
        this.target = val;
    }

    public boolean isFinished()
    {
        return ( Math.abs(lastError) < ERROR_TOLERANCE ); 
    }

    public double calculate (double currPos)
    {
        double error = target - currPos;

        double derivative = (error - lastError) / timer.seconds();

        integralSum += error * timer.seconds();

        double output = (error * kP) + (integralSum * kI) + (derivative * kD);

        lastError = error;

        timer.reset();

        return output;
    }

    
}
