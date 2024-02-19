package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CS.RT;
import org.firstinspires.ftc.teamcode.setup.CH;
import org.firstinspires.ftc.teamcode.setup.VP;

@Autonomous(name="CompAutoRedBack", group="Linear OpMode")

public class CompAutoRedBack extends LinearOpMode {
    public CH ch = null;
    private VP vp = null;
    private ElapsedTime stepTimer = new ElapsedTime();
    private String Location;

    @Override
    public void runOpMode() {
        telemetry.update();

        ch = new CH(hardwareMap, this);
        vp = new VP(hardwareMap, this);

        vp.initCompVision();

        ch.rightPincer.setPosition(0.5);

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
                ch.imuTurn(55);
                TelemetryStep("Move to left");
                ch.EncoderMove(RT.E_SPIKE_LEFT_RIGHT);

            } else if (Location == "right") {
                vp.DESIRED_TAG_ID = 6;
                TelemetryStep("Turn to right");
                ch.imuTurn(-35);
                TelemetryStep("Move to right");
                ch.EncoderMove(RT.E_SPIKE_LEFT_RIGHT);
            } else {
                vp.DESIRED_TAG_ID = 5;
                TelemetryStep("Move to Center");
                ch.EncoderMove(RT.E_SPIKE_LEFT_CENTER);
            }

            TelemetryStep("Back from spike mark");
            ch.moveRobot(-0.4, 0, 0);
            sleep(500);
            ch.moveRobot(0, 0, 0);

            TelemetryStep("Turn to backdrop");
            ch.imuTurn(100);

            vp.visionPortal.setActiveCamera(vp.webcam1);
            stepTimer.reset();

            TelemetryStep("Move april tag");
            ch.moveAprilTag(vp);

            TelemetryStep("Move to backdrop");
            ch.moveRobot(-0.4,0,0);
            sleep(1000);
            ch.moveRobot(0,0,0);

            TelemetryStep("Move arm up");
            ch.armMove(2100);
            TelemetryStep("Drop on backdrop ");
            ch.rightPincer.setPosition(0.85);
            sleep(2000);

            ch.armMove(0);
            sleep(2000);


        } // if active
    } // run op mode
    private void TelemetryStep(String step) {
        telemetry.addData("Step", step);
        telemetry.addData("prop location", Location);
        telemetry.update();
    }
} //linear op mode