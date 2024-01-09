package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.setup.CH;
import org.firstinspires.ftc.teamcode.setup.VP;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Autonomous(name="newAuto", group="Linear OpMode")

public class NewAuto extends LinearOpMode {
    final double DESIRED_DISTANCE = 11.0; //  this is how close the camera should get to the target (inches)
    private String CUP_POS = "Middle";
    private AprilTagDetection desiredTag = null;
    private CH ch = null;
    private VP vp = null;
    private static int DESIRED_TAG_ID = 5;
    private ElapsedTime stepTimer = new ElapsedTime();
    private boolean targetNotReached = true;

    private enum Step { //creating enum step stuff
        TENSOR_DETECT,
        TENSOR_MOVE_1,
        TENSOR_MOVE_2,
        TENSOR_MOVE_RIGHT,
        TENSOR_MOVE_MIDDLE,
        TENSOR_MOVE_LEFT,
    }
    Step currentStep = Step.TENSOR_DETECT;

    @Override
    public void runOpMode() {
        ch = new CH(hardwareMap);
        vp = new VP(hardwareMap);

        vp.initAprilTag();

        waitForStart();
        stepTimer.reset();
        while (opModeIsActive()) {

            switch(currentStep) {
                case TENSOR_DETECT:

                    if (stepTimer.milliseconds() < 7000) {
                        List<Recognition> currentRecognitions = vp.tfod.getRecognitions();

                        // Step through the list of recognitions and display info for each one.
                        for (Recognition recognition : currentRecognitions) {
                            vp.cupFound = true;
                            double x = (recognition.getLeft() + recognition.getRight()) / 2;

                            if (x < 600) {
                                CUP_POS = "right";
                            } else if (x > 1300) {
                                CUP_POS = "left";
                            } else {
                                CUP_POS = "middle";
                            }
                        }
                        if (vp.cupFound){
                            currentStep = Step.TENSOR_MOVE_1;
                        }

                    } else{ // if not detected
                        CUP_POS = "middle";
                     currentStep = Step.TENSOR_MOVE_1;
                    }
                    stepTimer.reset();
                    break;

                case TENSOR_MOVE_1:
                    if (stepTimer.milliseconds() < 500) {
                        ch.moveRobot(0.5,0, 0);
                    }
                    stepTimer.reset();
                    currentStep = Step.TENSOR_MOVE_2;
                    break;


                case TENSOR_MOVE_2:

                    if (stepTimer.milliseconds() < 250) {
                        ch.moveRobot(0,0.5, 0);
                    }

                    else {
                        ch.moveRobot(0, 0, 0);
                        sleep(200);

                        if (CUP_POS == "right"){
                            currentStep = Step.TENSOR_MOVE_RIGHT;
                        }
                        else if (CUP_POS == "left"){
                            currentStep = Step.TENSOR_MOVE_LEFT;
                        }
                        else {
                            currentStep = Step.TENSOR_MOVE_MIDDLE;
                        }
                    }

                    stepTimer.reset();
                    break;

                case TENSOR_MOVE_MIDDLE:
                    if (stepTimer.milliseconds() < 225) {
                        ch.imuMove(0.5,0, 0);
                    }

                    else {
                        ch.moveRobot(0, 0, 0);
                        currentStep = Step.TENSOR_MOVE_MIDDLE;
                        stepTimer.reset();
                    }
                    break;

                case TENSOR_MOVE_RIGHT:
                    if (stepTimer.milliseconds() < 225) {
                        ch.imuMove(0,-70, 0);
                    }
                    break;

            } // switch
            telemetry.addData("Step", currentStep);
            telemetry.update();



     //       vp.setManualExposure(6, 250);  // Use low exposure time to reduce motion blur


        } // opmode is active
    } // end



    public void moveAprilTag(){

        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)

        desiredTag = null;
        while (targetNotReached) {
            targetFound = false;
            List<AprilTagDetection> currentDetections = vp.aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {                     //  Check to see if we want to track towards this tag.
                        targetFound = true;                         // Yes, we want to use this tag.
                        desiredTag = detection;
                        break;  // don't look any further.
                    } else {
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);                        // This tag is in the library, but we do not want to track it right now.
                    }
                } else {
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);                    // This tag is NOT in the library, so we don't have enough information to track to it.
                }
            }

            // Tell the driver what we see, and what to do.
            if (targetFound) {
                telemetry.addData("\n>", "HOLD Left-Bumper to Drive to Target\n");
                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);

                double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                double headingError = desiredTag.ftcPose.bearing;
                double yawError = -desiredTag.ftcPose.yaw;

                if ((rangeError < 4) && (Math.abs(headingError) < 6) && (Math.abs(yawError) < 6)) {
                    drive = 0;
                    turn = 0;
                    strafe = 0;
                    targetNotReached = false;
                } else {
                    drive = Range.clip(rangeError * ch.SPEED_GAIN, -ch.MAX_AUTO_SPEED, ch.MAX_AUTO_SPEED);
                    turn = Range.clip(headingError * ch.TURN_GAIN, -ch.MAX_AUTO_TURN, ch.MAX_AUTO_TURN);
                    strafe = Range.clip(-yawError * ch.STRAFE_GAIN, -ch.MAX_AUTO_STRAFE, ch.MAX_AUTO_STRAFE);
                    telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
                }
                telemetry.update();

                // Apply desired axes motions to the drivetrain.
                ch.moveRobot(-drive, strafe, turn);
                sleep(10);
            }
            else {
                telemetry.addData("\n>", "Not found");
                ch.moveRobot(0,0,0);
            }
        }    telemetry.update();
    }
}
