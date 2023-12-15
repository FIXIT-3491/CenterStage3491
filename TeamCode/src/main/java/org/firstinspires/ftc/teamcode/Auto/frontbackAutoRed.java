/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.setup.CH;
import org.firstinspires.ftc.teamcode.setup.VP;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.Currency;
import java.util.List;
import java.util.concurrent.TimeUnit;


@Autonomous(name="frontbackAutoRed", group="Linear OpMode")

public class frontbackAutoRed extends LinearOpMode {

    private String CUP_POS = "Middle";

    final double DESIRED_DISTANCE = 11.0; //  this is how close the camera should get to the target (inches)

    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.2;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.2;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.2;   //  Clip the turn speed to this max value (adjust for your robot)
    private AprilTagDetection desiredTag = null;

    private CH ch = null;
    private VP vp = null;
    private boolean gamepadPressed = false;
    private static int DESIRED_TAG_ID = 5;
    private ElapsedTime runtime = new ElapsedTime();

    private boolean targetNotReached = true;

    @Override
    public void runOpMode() {

        ch = new CH(hardwareMap);
        vp = new VP(hardwareMap);

        vp.initAprilTag();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Gamepad A is front Gamepad B is back", ch.Front);
        telemetry.update();

        while (gamepadPressed == false) {
            if (gamepad1.a) {
                ch.Front = true;
                gamepadPressed = true;

            }
            if (gamepad1.b) {
                ch.Front = false;
                gamepadPressed = true;
            }
        }
        setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

        telemetry.addData("Front =", ch.Front);
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (vp.cupFound == false){
            List<Recognition> currentRecognitions = vp.tfod.getRecognitions();
            telemetry.addData("# Objects Detected", currentRecognitions.size());

            // Step through the list of recognitions and display info for each one.
            for (Recognition recognition : currentRecognitions) {
                vp.cupFound = true;
                double x = (recognition.getLeft() + recognition.getRight()) / 2;
                //double y = (recognition.getTop() + recognition.getBottom()) / 2;

                if(x<600){
                    CUP_POS = "left";
                }
                else if (x>1300) {
                    CUP_POS = "right";
                }
                else {
                    CUP_POS = "middle";
                }
            }
        }


            if (ch.Front == true) {

                telemetry.addData("Position", CUP_POS);
                ch.moveRobot(0.6, 0.15, 0);
                sleep(700);
                ch.moveRobot(0, 0, 0);
                sleep(500);

                if (CUP_POS == "right") {
                    DESIRED_TAG_ID = 6;
                    ch.moveRobot(0, 0, -0.5); // turn spike mark
                    sleep(350);
                    ch.moveRobot(0, 0, 0);
                    sleep(300);
                    ch.moveRobot(0.5, 0, 0); // drive spike mark
                    sleep(315);
                    ch.moveRobot(0, 0, 0);
                    sleep(200);
                    ch.moveRobot(-0.5, 0, 0); //drive away from spike mark
                    sleep(200);
                    ch.moveRobot(0, 0, 0.5); // turn to backdrop
                    sleep(900);
                    ch.moveRobot(0, 0, 0);

                } else if (CUP_POS == "left") {
                    DESIRED_TAG_ID = 4;
                    ch.moveRobot(0, 0, 0.5); // turn to spike mark
                    sleep(350);
                    ch.moveRobot(0, 0, 0);
                    sleep(300);
                    ch.moveRobot(0.5, 0, 0); // drive to spike mark
                    sleep(350);
                    ch.moveRobot(0, 0, 0);
                    sleep(200);
                    ch.moveRobot(-0.5, 0, 0);  // drive away from spike mark
                    sleep(200);
                    ch.moveRobot(0, 0, 0.5); // turn to backdrop
                    sleep(350);
                    ch.moveRobot(0, 0, 0);


                } else if (CUP_POS == "middle") {
                    DESIRED_TAG_ID = 5;
                    ch.moveRobot(0.5, 0, 0); // turn to spike mark
                    sleep(325);
                    ch.moveRobot(0, 0, 0);
                    sleep(200);
                    ch.moveRobot(-0.5, 0, 0); // drive away from spike mark
                    sleep(325);
                    ch.moveRobot(0,0,0);
                    sleep(200);
                    ch.moveRobot(0, 0, 0.5); // turn to backdrop
                    sleep(500);
                    ch.moveRobot(0, 0, 0);

                }
            }
            else {
                telemetry.addData("Position", CUP_POS);
                ch.moveRobot(0.6, 0.15, 0);
                sleep(700);
                ch.moveRobot(0, 0, 0);
                sleep(500);

                if (CUP_POS == "right") {
                    DESIRED_TAG_ID = 6;
                    ch.moveRobot(0, 0, -0.5); //turn to spike mark
                    sleep(350);
                    ch.moveRobot(0, 0, 0);
                    sleep(300);
                    ch.moveRobot(0.5, 0, 0); //drive to spike mark
                    sleep(315);
                    ch.moveRobot(0, 0, 0);
                    sleep(200);
                    ch.moveRobot(-0.5, 0, 0);  //drive away from spike mark
                    sleep(200);
                    ch.moveRobot(0, 0, 0.5); // turn to backdrop
                    sleep(900);
                    ch.moveRobot(0, 0, 0);



                } else if (CUP_POS == "left") {
                    DESIRED_TAG_ID = 4;
                    ch.moveRobot(0, 0, 0.5); // turn to spike mark
                    sleep(350);
                    ch.moveRobot(0, 0, 0);
                    sleep(300);
                    ch.moveRobot(0.5, 0, 0); // drive to spike mark
                    sleep(375);
                    ch.moveRobot(0, 0, 0);
                    sleep(200);
                    ch.moveRobot(-0.5, 0, 0); // drive away from spike mark
                    sleep(200);
                    ch.moveRobot(0, 0, 0.5); // turn to backdrop
                    sleep(350);
                    ch.moveRobot(0, 0, 0);



                } else if (CUP_POS == "middle") {
                    DESIRED_TAG_ID = 5;
                    ch.moveRobot(0.5, 0, 0); // drive to spike mark
                    sleep(325);
                    ch.moveRobot(0, 0, 0);
                    sleep(200);
                    ch.moveRobot(-0.5, 0, 0); // drive away from spike mark
                    sleep(200);
                    ch.moveRobot(0, 0, 0.5); // turn to backdrop
                    sleep(500);
                    ch.moveRobot(0, 0, 0);
                }
            }
            vp.visionPortal.setActiveCamera(vp.webcam1);
            moveAprilTag();
            sleep(1000);
            ch.moveRobot(-0.25,0,0); // move to backdrop

            sleep(600);
            ch.moveRobot(0,0,0);
            sleep(300);
            ch.hookArm.setPosition(ch.armPOS_2);
            sleep(400);
            ch.moveRobot(0,0,0);
            sleep(400);
            ch.hookArm.setPosition(ch.armMIN_POS);
            sleep(300);

        telemetry.update();

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

      //  }
    }

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
                    //  Check to see if we want to track towards this tag.
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTag = detection;
                        break;  // don't look any further.
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
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
                    drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                    turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                    strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
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
    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (vp.visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (vp.visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (vp.visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = vp.visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            //GainControl gainControl = vp.visionPortal.getCameraControl(GainControl.class);
            //gainControl.setGain(gain);
            sleep(20);
        }
    }
}