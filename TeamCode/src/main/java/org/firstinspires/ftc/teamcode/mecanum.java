package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.odometry.constants;
import org.firstinspires.ftc.teamcode.util.Encoder;

@TeleOp(name="mecanum", group="LinearOpmode")
public class mecanum extends OpMode {
    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();


    private MecanumDrive drive;

    private Motor fL, fR, bL, bR;
    private DcMotor lE = null;
    private DcMotor rE = null;
    private DcMotor cE = null;
    private GamepadEx driverOp;




    @Override
    public void init() {
        /* instantiate motors */
        fL = new Motor(hardwareMap, "frontLeft");
        fR = new Motor(hardwareMap, "frontRight");
        bL = new Motor(hardwareMap, "rearLeft");
        bR = new Motor(hardwareMap, "rearRight");
//        fL = hardwareMap.get(DcMotor.class, "frontLeft");
//        fR = hardwareMap.get(DcMotor.class, "frontRight");
//        bL = hardwareMap.get(DcMotor.class, "rearLeft");
//        bR = hardwareMap.get(DcMotor.class, "rearRight");

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



        // setting motors to power mode for testing
        fL.setRunMode(Motor.RunMode.RawPower);
        fR.setRunMode(Motor.RunMode.RawPower);
        bL.setRunMode(Motor.RunMode.RawPower);
        bR.setRunMode(Motor.RunMode.RawPower);
//        bR.setDirection(DcMotorSimple.Direction.REVERSE);
//        fL.setDirection(DcMotorSimple.Direction.REVERSE);

        fL.setInverted(true);
        fR.setInverted(true);
        drive = new MecanumDrive(fL, fR, bL, bR);
        driverOp = new GamepadEx(gamepad1);

    }

    @Override
    public void loop() {
         drive.driveRobotCentric(
              0.5*driverOp.getLeftX(),
                 0.5*driverOp.getLeftY(),
                 0.5*driverOp.getRightX()


         );
  /*      if (driverOp.getLeftY() == 1)
        {
            fL.setPower(0.5);
            fR.setPower(0.5);
            bL.setPower(0.5);
            bR.setPower(0.5);
        }
        else{
            fL.setPower(0);
            fR.setPower(0);
            bL.setPower(0);
            bR.setPower(0);
        }*/
        
        // run all motors in same direction to see which ones to reverse
        //fL.set(0.5);
//        fR.set(0.5);
//        bL.set(0.5);
//        bR.set(0.5);
        constants x = new constants();
        telemetry.addData("Left Stick Y", driverOp.getLeftY());
        telemetry.addData("Left Stick X", driverOp.getLeftX());
        telemetry.addData("Right Stick X", driverOp.getRightX());
        telemetry.addData("leftEnc", lE.getCurrentPosition());
        telemetry.addData("rightEnc", rE.getCurrentPosition());
        telemetry.addData("Center", cE.getCurrentPosition());
        telemetry.addData("Inches", x.encoderTicksToInches(lE.getCurrentPosition()));
        telemetry.update();

    }
}
