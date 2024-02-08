package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.setup.CH;
import org.firstinspires.ftc.teamcode.setup.VP;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Autonomous(name="EncoderForward", group="Linear OpMode")

public class EncoderForward extends LinearOpMode {
    private static int DESIRED_TAG_ID = 5;
    private AprilTagDetection desiredTag = null;
    private boolean targetNotReached = true;
    private String CUP_POS = "Middle";
    public CH ch = null;
    private VP vp = null;
    private ElapsedTime stepTimer = new ElapsedTime();

    double power = 0.1;
    boolean rampUp = true;


    @Override
    public void runOpMode() {
// hi!!!! -JAKE, NOT ON FIX IT
        ch = new CH(hardwareMap);
        vp = new VP(hardwareMap);

        vp.initCompVision();

        telemetry.addData("Status", "initialized ");
        telemetry.update();


        waitForStart();

        if (opModeIsActive()) {
                if (stepTimer.milliseconds() < 7000 && vp.cupFound == false) {
                    List<Recognition> currentRecognitions = vp.tfod.getRecognitions();

                    // Step through the list of recognitions and display info for each one.
                    for (Recognition recognition : currentRecognitions) {
                        vp.cupFound = true;
                        double x = (recognition.getLeft() + recognition.getRight()) / 2;
                        if (x < 200) {
                            CUP_POS = "left";
                            DESIRED_TAG_ID = 4;
                        } else if (x > 430) {
                            CUP_POS = "right";
                            DESIRED_TAG_ID = 6;
                        } else {
                            CUP_POS = "middle";
                            DESIRED_TAG_ID = 5;
                        }
                    }
                } else { //  not detected
                    CUP_POS = "middle";
                }

                ch.EncoderMove(750);

                if (CUP_POS == "left") {
                    ch.imuTurn(-57);
                    ch.EncoderMove(500);
                } else if (CUP_POS == "right") {
                    ch.imuTurn(50);
                    ch.EncoderMove(500);
                } else {
                    ch.EncoderMove(800);
                }

                ch.moveRobot(-0.4, 0, 0);
                sleep(500);


                    ch.imuTurn(100);

            } // if active
        } // run op mode

    } //linear op mode
