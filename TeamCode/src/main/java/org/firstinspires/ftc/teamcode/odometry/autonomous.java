package org.firstinspires.ftc.teamcode.odometry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@Autonomous(name="autonomous", group="LinearOpmode")
public class autonomous extends LinearOpMode {
    private DcMotor fL, fR, bL, bR;
    private DcMotor lE = null;
    private DcMotor rE = null;
    private DcMotor cE = null;
    // pid control test
    private static double kU = 0.00045;
    private static double TU = 0.86957;
    public static double kP = 0.2*kU;
    public static double kI = 0.4*(kU/TU)*0.5;
    public static double kD = 0.066*kU*TU;
    public static double kF = 0;
    public static double distance =20;

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        fL = hardwareMap.get(DcMotor.class, "frontLeft");
        fR = hardwareMap.get(DcMotor.class, "frontRight");
        bL = hardwareMap.get(DcMotor.class, "rearLeft");
        bR = hardwareMap.get(DcMotor.class, "rearRight");
        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lE = hardwareMap.get(DcMotor.class, "leftEnc");
        rE = hardwareMap.get(DcMotor.class, "rightEnc");
        cE = hardwareMap.get(DcMotor.class, "centerEnc");




        lE.setDirection(DcMotor.Direction.REVERSE);
        rE.setDirection(DcMotor.Direction.REVERSE);
        cE.setDirection(DcMotor.Direction.REVERSE);
        lE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        cE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        bR.setDirection(DcMotorSimple.Direction.REVERSE);
        fL.setDirection(DcMotorSimple.Direction.REVERSE);





        waitForStart();




        drive x = new drive(90, 10);
        double angle = 0.5*Math.PI;
        double output = 0;
        telemetry.addData("Position", constants.encoderTicksToInches((int) (0.5*(lE.getCurrentPosition() + rE.getCurrentPosition()))));
        telemetry.addData("Expected position", constants.inchesToEncoderTicks(distance));
        telemetry.addData("Output", output);
        telemetry.update();
        double frontLeft = Math.sin(angle + 0.25*Math.PI);
        double frontRight = Math.sin(angle - 0.25*Math.PI);
        double rearLeft =  Math.sin(angle - 0.25*Math.PI);
        double rearRight = Math.sin(angle + 0.25*Math.PI);
        sleep(5000);
        //front right = back left
        //front left = back right
//        fL.setPower(0.25*frontLeft);
//        fR.setPower(0.25*frontRight);
//        bL.setPower(0.25*rearLeft);
//        bR.setPower(0.25*rearRight);




        PIDFController pidfController = new PIDFController(kP, kI, kD, kF);
        pidfController.reset();
        pidfController.setSetPoint(constants.inchesToEncoderTicks(distance));
        pidfController.setTolerance(100, 10);
        pidfController.setIntegrationBounds(-0.25/kI, 0.25/kI);

        int lETicks = lE.getCurrentPosition();
        int rETicks = rE.getCurrentPosition();
//        while(opModeIsActive()){
//            while (!pidfController.atSetPoint() && opModeIsActive())
//            {
//                double output = pidfController.calculate(lE.getCurrentPosition());
//                fL.setPower(0.5*output);
//                fR.setPower(0.5*output);
//                bL.setPower(0.5*output);
//                bR.setPower(0.5*output);
////            telemetry.addData("leftEnc", lE.getCurrentPosition());
////            telemetry.addData("Center", cE.getCurrentPosition());
////            telemetry.addData("rightEnc", rE.getCurrentPosition());
////            telemetry.addData("Error", pidfController.getPositionError());
//                telemetry.addData("Position", 10*constants.encoderTicksToInches(lE.getCurrentPosition()));
//                telemetry.addData("Expected position", 10*distance);
//                telemetry.update();
//
//            }
//            fL.setPower(0);
//            fR.setPower(0);
//            bL.setPower(0);
//            bR.setPower(0);
//            pidfController.setSetPoint(constants.inchesToEncoderTicks(-distance));
//            while (!pidfController.atSetPoint() && opModeIsActive())
//            {
//                double output = pidfController.calculate(lE.getCurrentPosition());
//                fL.setPower(0.5*output);
//                fR.setPower(0.5*output);
//                bL.setPower(0.5*output);
//                bR.setPower(0.5*output);
////            telemetry.addData("leftEnc", lE.getCurrentPosition());
////            telemetry.addData("Center", cE.getCurrentPosition());
////            telemetry.addData("rightEnc", rE.getCurrentPosition());
////            telemetry.addData("Error", pidfController.getPositionError());
//                telemetry.addData("Position", 10*constants.encoderTicksToInches(lE.getCurrentPosition()));
//                telemetry.addData("Expected position", 10*distance);
//                telemetry.update();
//
//            }
//            fL.setPower(0);
//            fR.setPower(0);
//            bL.setPower(0);
//            bR.setPower(0);
//
//        }
//        output = 0;
//        while (!pidfController.atSetPoint() && lE.getCurrentPosition() < 0.4*constants.inchesToEncoderTicks(distance))
//        {
//            output = output +0.05;
//            fL.setPower(output);
//            fR.setPower(output);
//            bL.setPower(output);
//            bR.setPower(output);
//            telemetry.addData("Position", 10*constants.encoderTicksToInches(lE.getCurrentPosition()));
//            telemetry.addData("Expected position", 10*distance);
////            telemetry.addData("Output", output);
//            telemetry.update();
//        }
        double Loutput;
        double ROutput;
        while (!pidfController.atSetPoint() && opModeIsActive())
        {
            pidfController.setPIDF(kP, kI, kD, kF);
             output = pidfController.calculate(lE.getCurrentPosition());
            fL.setPower(output);
            fR.setPower(output);
            bL.setPower(output);
            bR.setPower(output);
//            telemetry.addData("leftEnc", lE.getCurrentPosition());
//            telemetry.addData("Center", cE.getCurrentPosition());
//            telemetry.addData("rightEnc", rE.getCurrentPosition());
//            telemetry.addData("Error", pidfController.getPositionError());
            telemetry.addData("Position", 10*constants.encoderTicksToInches(lE.getCurrentPosition()));
            telemetry.addData("Expected position", 10*distance);
//            telemetry.addData("Output", output);
            telemetry.update();

        }

        fL.setPower(0);
        fR.setPower(0);
        bL.setPower(0);
        bR.setPower(0);

        //constants y = new constants();
////
//        telemetry.addData("le", fL.getPower());
//        telemetry.addData("front right", fR.getPower());
//        telemetry.addData("rear Left", bL.getPower());
//        telemetry.addData("rear right", bR.getPower());
//        telemetry.addData("at set point", pidfController.atSetPoint());
        //telemetry.addData("Inches", y.encoderTicksToInches(lE.getCurrentPosition()));
        telemetry.update();
    }


}
