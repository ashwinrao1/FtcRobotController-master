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
@Autonomous(name="TrackingTest", group="LinearOpmode")
public class trackingTest extends LinearOpMode {

    private DcMotor lE = null;
    private DcMotor rE = null;
    private DcMotor cE = null;
    private int lEInitial;
    private int rEInitial;
    private int cEInitial;


    @Override
    public void runOpMode() throws InterruptedException {
        tracking tracker = new tracking(4.0625, 4.0625, 5.34375, 0);
        lE = hardwareMap.get(DcMotor.class, "leftEnc");
        rE = hardwareMap.get(DcMotor.class, "rightEnc");
        cE = hardwareMap.get(DcMotor.class, "centerEnc");

        // setting encoders to correct direction
        lE.setDirection(DcMotor.Direction.REVERSE);
        rE.setDirection(DcMotor.Direction.REVERSE);
        cE.setDirection(DcMotor.Direction.REVERSE);

        // resetting encoders
        lE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        cE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lEInitial = lE.getCurrentPosition();
        rEInitial = rE.getCurrentPosition();
        cEInitial = cE.getCurrentPosition();
        waitForStart();

        while (opModeIsActive())
        {

            tracker.calculateValues(lE.getCurrentPosition(), rE.getCurrentPosition(), cE.getCurrentPosition(), lEInitial, rEInitial, cEInitial);


            telemetry.addData("X", tracker.d_1[0]);
            telemetry.addData("Y", tracker.d_1[1]);
            telemetry.addData("theta", tracker.theta_1*180/Math.PI);

            telemetry.update();
        }
    }


}

