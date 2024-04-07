package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.setup.Constants.CS;
import org.firstinspires.ftc.teamcode.setup.CH;
import org.firstinspires.ftc.teamcode.setup.VP;

@Autonomous(name="CompAutoBlueBack", group="Linear OpMode")

public class CompAutoBlueBack extends LinearOpMode {
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
                vp.DESIRED_TAG_ID = 1;
                TelemetryStep("Turn to left");
                ch.imuTurn(50);
                TelemetryStep("Move to left");
                ch.EncoderMove(CS.E_SPIKE_RIGHT);

            } else if (Location == "right") {
                vp.DESIRED_TAG_ID = 3;
                TelemetryStep("Turn to right");
                ch.imuTurn(-37);
                TelemetryStep("Move to right");
                ch.EncoderMove(CS.E_SPIKE_RIGHT);
            } else if (Location == "not detected") {
                vp.DESIRED_TAG_ID = 3;
                TelemetryStep("Turn to right");
                ch.imuTurn(-41);
                TelemetryStep("Move to right");
                ch.EncoderMove(CS.E_SPIKE_RIGHT);

            } else {
                vp.DESIRED_TAG_ID = 2;
                TelemetryStep("Move to Center");
                ch.EncoderMove(CS.E_SPIKE_LEFT_CENTER);
            }

            TelemetryStep("Back from spike mark");
            ch.moveRobot(-0.4, 0, 0);
            sleep(500);
            ch.moveRobot(0, 0, 0);

            TelemetryStep("Turn to backdrop");
            ch.imuTurn(-100);
            vp.visionPortal.setActiveCamera(vp.webcam1);
            stepTimer.reset();

            ch.moveRobot(-0.3,-0.3,0);
            sleep(500);
            ch.moveRobot(0,0,0);

            TelemetryStep("Move april tag");
            ch.moveAprilTag(vp);

            TelemetryStep("Move to backdrop");
            ch.moveRobot(-0.4,0,0);
            sleep(1200);
            ch.moveRobot(0,0,0);

            TelemetryStep("Move arm up");
            ch.armMove(2100);
            TelemetryStep("Drop on backdrop ");
            ch.rightPincer.setPosition(CS.C_RIGHT_OPEN);
            sleep(400);

            TelemetryStep("Lower Arm");
            ch.armMove(0);
            sleep(500);
            TelemetryStep("Close Pincer");
            ch.rightPincer.setPosition(CS.C_RIGHT_CLOSE);
            TelemetryStep("Drive off backdrop");
            ch.EncoderMove(400);
            TelemetryStep("turn to 0");
            ch.imuTurn(-179);
            TelemetryStep("move to park");
            ch.EncoderMove(1400);
            TelemetryStep("park ");
            ch.moveRobot(0,-0.5,0);
            sleep(1000);
            ch.moveRobot(0,0,0);


        } // if active
    } // run op mode
    private void TelemetryStep(String step) {
        telemetry.addData("Step", step);
        telemetry.addData("prop location", Location);
        telemetry.update();
    }
    private void YellowPixel(){
        vp.visionPortal.setActiveCamera(vp.webcam1);
        stepTimer.reset();

        TelemetryStep("Move april tag");
        ch.moveAprilTag(vp);
        TelemetryStep("Move to backdrop");
        ch.moveRobot(-0.4,0,0);
        sleep(1200);
        ch.moveRobot(0,0,0);
        TelemetryStep("Move arm up");
        ch.armMove(2100);
        TelemetryStep("Drop on backdrop ");
        ch.rightPincer.setPosition(CS.C_RIGHT_OPEN);
        sleep(400);
        TelemetryStep("Lower Arm");
        ch.armMove(0);
        sleep(500);
        TelemetryStep("Close Pincer");
        ch.rightPincer.setPosition(CS.C_RIGHT_CLOSE);
        TelemetryStep("Drive off backdrop");
        ch.EncoderMove(400);
        TelemetryStep("turn to 0");
        ch.imuTurn(179);
        TelemetryStep("move to park");
        ch.EncoderMove(1200);
        TelemetryStep("park ");
        ch.moveRobot(0,0.5,0);
        sleep(1000);
        ch.moveRobot(0,0,0);
    }
} //linear op mode