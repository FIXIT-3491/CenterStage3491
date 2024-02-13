package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.setup.CH;
import org.firstinspires.ftc.teamcode.setup.VP;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Autonomous(name="CompAutoBlueBack", group="Linear OpMode")

public class CompAutoBlueBack extends LinearOpMode {
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

        telemetry.addData("Status", "initialized ");
        telemetry.update();

        waitForStart();

        stepTimer.reset();
        if (opModeIsActive())
        {
            vp.TensorDetect();
            Location = vp.TensorDetect();

            ch.EncoderMove(750);

            if (Location == "left") {
                ch.imuTurn(57);
                ch.EncoderMove(ch.SPIKE_LEFT_RIGHT);
            } else if (Location == "right") {
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