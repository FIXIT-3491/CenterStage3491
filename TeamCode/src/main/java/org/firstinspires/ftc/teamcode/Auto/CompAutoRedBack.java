package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.setup.Constants.CS;
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

//        ch.imu.resetYaw();

        ch = new CH(hardwareMap, this);
        vp = new VP(hardwareMap, this);

        vp.initCompVision();

        ch.rightPincer.setPosition(0.5);
        ch.wrist.setPosition(CS.WRIST_UP);

        telemetry.addData("Status", "initialized ");
        telemetry.update();

        waitForStart();
        ch.wrist.setPosition(CS.WRIST_DOWN);

        stepTimer.reset();
        if (opModeIsActive())
        {

            TelemetryStep("TensorDetect");
            Location = vp.TensorDetect();
            TelemetryStep("Move forward");
            ch.wrist.setPosition(0.13);
            ch.EncoderMove(750);


            if (Location == "left") {
                vp.DESIRED_TAG_ID = 4;
                TelemetryStep("Turn to left");
                ch.imuTurn(44);
                TelemetryStep("Move to left");
                ch.EncoderMove(CS.E_SPIKE_LEFT);
                BackFromSpike(1200);

            } else if (Location == "right") {
                vp.DESIRED_TAG_ID = 6;
                TelemetryStep("Turn to right");
                ch.imuTurn(-32);
                TelemetryStep("Move to right");
                ch.EncoderMove(CS.E_SPIKE_RIGHT);
                BackFromSpike(550);

            } else {
                vp.DESIRED_TAG_ID = 5;
                TelemetryStep("Move to Center");
                ch.EncoderMove(CS.E_SPIKE_LEFT_CENTER);
                BackFromSpike(600);
            }


            TelemetryStep("Turn to backdrop");
            ch.imuTurn(-90);


            YellowPixel();

        } // if active
    } // run op mode
    private void TelemetryStep(String step) {
        telemetry.addData("Step", step);
        telemetry.addData("prop location", Location);
        telemetry.update();
    }
    private void BackFromSpike(int amount){
        ch.wrist.setPosition(CS.WRIST_UP);
        TelemetryStep("Back from spike mark");
        ch.moveRobot(-0.5, 0, 0);
        sleep(amount);
        ch.moveRobot(0, 0, 0);
    }
    private void YellowPixel(){
        stepTimer.reset();
        ch.wrist.setPosition(0.15);

        TelemetryStep("Move april tag");
        ch.moveAprilTag(vp);
        ch.dropPixel1();
        ch.EncoderMove(300);

    }

} //linear op mode

//"hehehe" - Ryan