package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.setup.CH;
import org.firstinspires.ftc.teamcode.setup.VP;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Autonomous(name="EncoderForward", group="Linear OpMode")

public class PurplePixelAutoRed extends LinearOpMode {
    public CH ch = null;
    private VP vp = null;
    private ElapsedTime stepTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
// hi!!!! -JAKE, NOT ON FIX IT
        ch = new CH(hardwareMap);
        vp = new VP(hardwareMap);

        vp.initCompVision();

        telemetry.addData("Status", "initialized ");
        telemetry.update();

        waitForStart();

        stepTimer.reset();
        if (opModeIsActive())
        {

            vp.TensorDetect();
//            telemetry.addData("cup poz", vp.CUP_POS);
//            telemetry.update();
            ch.EncoderMove(750);

            if (vp.CUP_POS == "left") {
                ch.imuTurn(57);
                ch.EncoderMove(ch.SPIKE_LEFT_RIGHT);
            } else if (vp.CUP_POS == "right") {
                ch.imuTurn(-50);
                ch.EncoderMove(ch.SPIKE_LEFT_RIGHT);
            } else {
                ch.EncoderMove(ch.SPIKE_LEFT_CENTER);
            }
            ch.moveRobot(-0.4, 0, 0);
            sleep(500);
            ch.moveRobot(0, 0, 0);
        } // if active
    } // run op mode
} //linear op mode