package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants.RT;
import org.firstinspires.ftc.teamcode.setup.CH;
import org.firstinspires.ftc.teamcode.setup.VP;

@Autonomous(name="PurplePixelAutoRed", group="Linear OpMode")

public class PurplePixelAutoRed extends LinearOpMode {
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

        ch.rightPincer.setPosition(0.57);

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
                ch.imuTurn(-33);
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

            TelemetryStep("Move to backdrop");
            ch.imuTurn(100);


        } // if active
    } // run op mode
    private void TelemetryStep(String step) {
        telemetry.addData("Step", step);
        telemetry.addData("prop location", Location);
        telemetry.update();
    }
} //linear op mode