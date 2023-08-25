package org.firstinspires.ftc.teamcode.odometry;


import com.arcrobotics.ftclib.controller.PController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="autonomous", group="LinearOpmode")
public class autonomous extends OpMode {
    private DcMotor fL, fR, bL, bR;
    private DcMotor lE = null;
    private DcMotor rE = null;
    private DcMotor cE = null;


    @Override
    public void init() {
        fL = hardwareMap.get(DcMotor.class, "frontLeft");
        fR = hardwareMap.get(DcMotor.class, "frontRight");
        bL = hardwareMap.get(DcMotor.class, "rearLeft");
        bR = hardwareMap.get(DcMotor.class, "rearRight");

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








    }
    @Override
    public void loop() {
        drive x = new drive(90, 10);
        double angle = 1.5*Math.PI;

        double frontLeft = Math.sin(angle + 0.25*Math.PI);
        double frontRight = Math.sin(angle - 0.25*Math.PI);
        double rearLeft =  Math.sin(angle - 0.25*Math.PI);
        double rearRight = Math.sin(angle + 0.25*Math.PI);

        //front right = back left
        //front left = back right
        fL.setPower(0.5*frontLeft);
        fR.setPower(0.5*frontRight);
        bL.setPower(0.5*rearLeft);
        bR.setPower(0.5*rearRight);



        // pid control test
        double kP = 0;
        double kI = 0;
        double kD = 0;
        double kF = 0;
        PIDFController pidfController = new PIDFController(kP, kI, kD, kF);

        while (!pidfController.atSetPoint())
        {
            double output = pidfController.calculate(fL.getCurrentPosition());
            fL.setPower(output);
            DcMotor velocity = (DcMotor)fL;

        }




        //constants y = new constants();
//        telemetry.addData("leftEnc", lE.getCurrentPosition());
//        telemetry.addData("Center", cE.getCurrentPosition());
//        telemetry.addData("rightEnc", rE.getCurrentPosition());
        telemetry.addData("front Left", fL.getPower());
        telemetry.addData("front right", fR.getPower());
        telemetry.addData("rear Left", bL.getPower());
        telemetry.addData("rear right", bR.getPower());

        //telemetry.addData("Inches", y.encoderTicksToInches(lE.getCurrentPosition()));
        telemetry.update();
    }
}
