package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants.CS;
import org.firstinspires.ftc.teamcode.setup.CH;
import org.firstinspires.ftc.teamcode.setup.VP;

@Autonomous(name="CompAutoRedFront", group="Linear OpMode")

public class CompAutoRedFront extends LinearOpMode {
    public CH ch = null;
    private VP vp = null;
    private ElapsedTime stepTimer = new ElapsedTime();
    private String Location;

    @Override
    public void runOpMode() {
        ch.imu.resetYaw();

        ch = new CH(hardwareMap, this);
        vp = new VP(hardwareMap, this);

        vp.initCompVision();

        ch.rightPincer.setPosition(CS.C_RIGHT_CLOSE);

        telemetry.addData("Status", "initialized ");
        telemetry.update();

        waitForStart();

        stepTimer.reset();
        if (opModeIsActive())
        {
            TelemetryStep("TensorDetect");
            Location = vp.TensorDetect();
            TelemetryStep("Move forward");
            ch.EncoderMove(750);

            if (Location == "left") {
                vp.DESIRED_TAG_ID = 4;
                TelemetryStep("Turn to left");
                ch.imuTurn(52);
                TelemetryStep("Move to left");
                ch.EncoderMove(CS.E_SPIKE_LEFT_RIGHT);

            } else if (Location == "right") {
                vp.DESIRED_TAG_ID = 6;
                TelemetryStep("Turn to right");
                ch.imuTurn(-33);
                TelemetryStep("Move to right");
                ch.EncoderMove(CS.E_SPIKE_LEFT_RIGHT);
            } else {
                vp.DESIRED_TAG_ID = 5;
                TelemetryStep("Move to Center");
                ch.EncoderMove(CS.E_SPIKE_LEFT_CENTER);
            }

            TelemetryStep("Back from spike mark");
            ch.moveRobot(-0.4, 0, 0);
            sleep(500);
            ch.moveRobot(0, 0, 0);

            TelemetryStep("Move to backdrop");
            ch.imuTurn(100);

            ch.EncoderMove(750);
            ch.imuTurn(0);
            ch.EncoderMove(1500);
            ch.imuTurn(-87);
            ch.EncoderMove(1000);
            ch.moveRobot(0.8,0,0);
            sleep(1200);
            ch.moveRobot(0,0,0);
            vp.visionPortal.setActiveCamera(vp.webcam1);
            ch.imuTurn(60);
            ch.moveRobot(-0.5,0.5,0.05); sleep(700); ch.moveRobot(0,0,0);

            ch.moveAprilTag(vp);
            ch.moveRobot(-0.4,0,0);
            sleep(1200);
            ch.moveRobot(0,0,0);
            ch.armMove(2100);
            sleep(200);
            ch.rightPincer.setPosition(0.85);
            sleep(400);
            ch.armMove(0);
        } // if active
    } // run op mode
    private void TelemetryStep(String step) {
        telemetry.addData("Step", step);
        telemetry.addData("prop location", Location);
        telemetry.update();
    }
} //linear op mode