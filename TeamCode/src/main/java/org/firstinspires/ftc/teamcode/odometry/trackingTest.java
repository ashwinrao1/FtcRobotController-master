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


    @Override
    public void runOpMode() throws InterruptedException {
        tracking tracker = new tracking(2.84375, 2.84375, 5.34375, 0);
        lE = hardwareMap.get(DcMotor.class, "leftEnc");
        rE = hardwareMap.get(DcMotor.class, "rightEnc");
        cE = hardwareMap.get(DcMotor.class, "centerEnc");

        waitForStart();

        while (opModeIsActive())
        {
            tracker.calculateValues(lE.getCurrentPosition(), rE.getCurrentPosition(), cE.getCurrentPosition());


            telemetry.addData("X", tracker.d_1[0]);
            telemetry.addData("Y", tracker.d_1[1]);
            telemetry.addData("theta", tracker.theta_1);

            telemetry.update();
        }
    }


}

