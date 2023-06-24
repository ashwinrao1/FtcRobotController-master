package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="mecanum", group="LinearOpmode")
public class mecanum extends OpMode {
    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();


    private MecanumDrive drive;
    private Motor fL, fR, bL, bR;
    private GamepadEx driverOp;




    @Override
    public void init() {
        /* instantiate motors */
        fL = new Motor(hardwareMap, "frontLeft");

        fR = new Motor(hardwareMap, "frontRight");
        bL = new Motor(hardwareMap, "rearLeft");
        bR = new Motor(hardwareMap, "rearRight");
        bL.setInverted(true);
        bR.setInverted(true);
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
        telemetry.addData("Left Stick Y", driverOp.getLeftY());
        telemetry.addData("Left Stick X", driverOp.getLeftX());
        telemetry.addData("Right Stick X", driverOp.getRightX());
        telemetry.update();

    }
}
