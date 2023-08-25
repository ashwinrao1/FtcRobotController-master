package org.firstinspires.ftc.teamcode.odometry;
import com.arcrobotics.ftclib.controller.PController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.util.ElapsedTime;


public class PIDController {

    double kP;
    double kI;
    double kD;
    double kF;
    double integralSum;
    double lastError;
    double target;
    ElapsedTime timer;
    boolean isFinished;

    final double ERROR_TOLERANCE = 0.01;
    
    
//    public PIDController (double kP, double kI, double kD)
//    {
//        this.kP = kP;
//        this.kI = kI;
//        this.kD = kD;
//        this.integralSum = 0;
//        this.lastError = 0;
//        this.target = 0;
//        this.timer = new ElapsedTime();
//        this.isFinished = false;
//    }
//
//    public void setTarget(double val)
//    {
//        this.target = val;
//    }
//
//    public boolean isFinished()
//    {
//        return ( Math.abs(lastError) < ERROR_TOLERANCE );
//    }

    public double calculate (double currPos)
    {


        // Creates a PIDFController with gains kP, kI, kD, and kF
        PIDFController pidf = new PIDFController(kP, kI, kD, kF);
        // set our gains to some value
        pidf.setP(0.37);
        pidf.setI(0.05);
        pidf.setD(1.02);


        // Calculates the output of the PIDF algorithm based on sensor
// readings. Requires both the measured value
// and the desired setpoint
//        double output = pidf.calculate(
//                motor.getCurrentPosition(), setpoint
//        );

        /*
         * A sample control loop for a motor
         */
        PController pController = new PController(kP);

// We set the setpoint here.
// Now we don't have to declare the setpoint
// in our calculate() method arguments.
        pController.setSetPoint(1200);

// perform the control loop
        /*
         * The loop checks to see if the controller has reached
         * the desired setpoint within a specified tolerance
         * range
         */
//        while (!pController.atSetPoint()) {
//            double output = pController.calculate(
//                    m_motor.getCurrentPosition()  // the measured value
//            );
//            m_motor.setVelocity(output);
//        }
//        m_motor.stopMotor(); // stop the motor

// NOTE: motors have internal PID control
        return 5.0;
    }

    
}
