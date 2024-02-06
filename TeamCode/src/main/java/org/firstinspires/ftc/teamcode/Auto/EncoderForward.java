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

    double  power   = 0.1;
    boolean rampUp  = true;


    @Override
    public void runOpMode() {
// hi!!!! -JAKE, NOT ON FIX IT
        ch = new CH(hardwareMap);
        vp = new VP(hardwareMap);

        vp.initCompVision();

        telemetry.addData("Status","initialized ");
        telemetry.update();


        waitForStart();

        if (opModeIsActive()){
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
                        DESIRED_TAG_ID =5;
                    }
                }
            } else{ //  not detected
                CUP_POS = "middle";
            }

            EncoderMove(750);


            stepTimer.reset();
            while (stepTimer.milliseconds() < 1000) {
                if (CUP_POS == "left") {
                    ch.imuMove(0, -57);
                    EncoderMove(500);
                }
                else if (CUP_POS == "right") {
                        ch.imuMove(0, 50);
                    EncoderMove(500);
                }
                else {
                    EncoderMove(800);
                }
            }

            ch.moveRobot(-0.4,0,0);
            sleep(500);
            stepTimer.reset();
            while (stepTimer.milliseconds() < 2000) {
                ch.imuMove(0,100
                );
            }
        } // if active
    } // run op mode

    public void TensorDetections(){
        if (stepTimer.milliseconds() < 7000) {
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
                    DESIRED_TAG_ID =5;
                }
            }
            if (vp.cupFound){
            }

        } else{ //  not detected
            CUP_POS = "middle";
        }

    }
    public void EncoderMove(int targetPosition) {

        ch.backRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ch.backRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (ch.backRDrive.getCurrentPosition() < targetPosition){

            if (ch.backRDrive.getCurrentPosition() > targetPosition*0.5 && rampUp) {
                rampUp = !rampUp;   // Switch ramp direction
            }

            if (rampUp) {
                // Keep stepping up until we hit the max value.
                power += ch.E_INCREMENT;
                if (power >= ch.E_MAX_POWER) {
                    power = ch.E_MAX_POWER;
                }
            }
            else {
                // Keep stepping down until we hit the min value.
                power -= ch.E_INCREMENT;
                if (power <= ch.E_MIN_POWER) {
                    power = ch.E_MIN_POWER;
                    // rampUp = !rampUp;  // Switch ramp direction
                }
            }

            ch.moveRobot(power,0,0);

            telemetry.addData("Encoder poz",ch.backRDrive.getCurrentPosition() );
            telemetry.addData("motor power",power);
            telemetry.update();

            sleep(ch.E_CYCLE_MS);

        } // while
        ch.moveRobot(0,0,0);
    }//public void

} //linear op mode
