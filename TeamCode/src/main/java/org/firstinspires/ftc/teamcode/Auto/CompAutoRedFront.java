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

@Autonomous(name="CompAutoRedFront", group="Linear OpMode")

public class CompAutoRedFront extends LinearOpMode {
    private String CUP_POS = "Middle";
    public CH ch = null;
    public VP vp = null;
    private ElapsedTime stepTimer = new ElapsedTime();

        @Override
        public void runOpMode () {
// hi!!!! -JAKE, NOT ON FIX IT
            ch = new CH(hardwareMap);
            vp = new VP(hardwareMap);

            vp.initCompVision();

            telemetry.addData("Status", "initialized ");
            telemetry.update();

            waitForStart();

            stepTimer.reset();
            if (opModeIsActive()) {

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

                ch.imuTurn(100);
                vp.visionPortal.setActiveCamera(vp.webcam1);
                moveAprilTag();

            } // if active
        } // run op mode

    public void moveAprilTag(){

        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)

        vp.desiredTag = null;
        while (vp.targetNotReached) {
            targetFound = false;
            List<AprilTagDetection> currentDetections = vp.aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    if ((vp.DESIRED_TAG_ID < 0) || (detection.id == vp.DESIRED_TAG_ID)) {                     //  Check to see if we want to track towards this tag.
                        targetFound = true;                         // Yes, we want to use this tag.
                        vp.desiredTag = detection;
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
                telemetry.addData("Found", "ID %d (%s)", vp.desiredTag.id, vp.desiredTag.metadata.name);
                telemetry.addData("Range", "%5.1f inches", vp.desiredTag.ftcPose.range);
                telemetry.addData("Bearing", "%3.0f degrees", vp.desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw", "%3.0f degrees", vp.desiredTag.ftcPose.yaw);


                double rangeError = (vp.desiredTag.ftcPose.range - vp.DESIRED_DISTANCE);
                double headingError = vp.desiredTag.ftcPose.bearing;
                double yawError = -vp.desiredTag.ftcPose.yaw;

                if ((rangeError < 4) && (Math.abs(headingError) < 6) && (Math.abs(yawError) < 6)) {
                    drive = 0;
                    turn = 0;
                    strafe = 0;
                    vp.targetNotReached = false;
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
    } //linear op mode