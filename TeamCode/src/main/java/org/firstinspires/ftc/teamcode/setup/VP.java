package org.firstinspires.ftc.teamcode.setup;

import static android.os.SystemClock.sleep;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;


import java.util.List;
import java.util.concurrent.TimeUnit;

public class VP {
    public VisionPortal visionPortal;

    public AprilTagProcessor aprilTag;
    public WebcamName webcam1, webcam2;


    private static int DESIRED_TAG_ID = 5;


    public String CUP_POS = "Middle";
    private ElapsedTime stepTimer = new ElapsedTime();


    public static final String TFOD_MODEL_ASSET = "rookDetection.tflite";
    public static final String[] LABELS = {"rook"};
    public TfodProcessor tfod;
    public boolean cupFound = false;
    public CH ch = null;

    public VP(HardwareMap hardwareMap) {
        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcam2 = hardwareMap.get(WebcamName.class, "Webcam 2");
    }

    public void initCompVision() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder()
          //      .setLensIntrinsics(1439.41944052, 1439.41944052, 970.51421863, 537.612825157)  //logitech c920
                .build();
        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        CameraName switchableCamera = ClassFactory.getInstance()
                .getCameraManager().nameForSwitchableCamera(webcam2, webcam1);

        tfod = new TfodProcessor.Builder()
                .setModelAssetName(TFOD_MODEL_ASSET)
                .setModelLabels(LABELS)
                .build();
        tfod.setMinResultConfidence(0.80f);

        visionPortal = new VisionPortal.Builder()
                .setCamera(switchableCamera)
                .setCameraResolution(new Size(640, 480))
                .addProcessor(aprilTag)
                .addProcessor(tfod)
                .build();

    }

    public void setManualExposure(int exposureMS, int gain) {

            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            //GainControl gainControl = vp.visionPortal.getCameraControl(GainControl.class);
            //gainControl.setGain(gain);
        }

    public void TensorDetect(){
        stepTimer.reset();
        while (stepTimer.milliseconds() < 7000 && !cupFound) {
            List<Recognition> currentRecognitions = tfod.getRecognitions();

            // Step through the list of recognitions and display info for each one.
            for (Recognition recognition : currentRecognitions) {
                cupFound = true;
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
        }
        if(!cupFound) { //  not detected
            CUP_POS = "middle";
        }
    }
    public void moveAprilTag(){

        final double DESIRED_DISTANCE = 11.0; //  this is how close the camera should get to the target (inches)
        boolean targetNotReached = true;
        AprilTagDetection desiredTag = null;

        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)

        desiredTag = null;
        while (targetNotReached) {
            targetFound = false;
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {                     //  Check to see if we want to track towards this tag.
                        targetFound = true;                         // Yes, we want to use this tag.
                        desiredTag = detection;
                        break;  // don't look any further.
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                }
            }

            // Tell the driver what we see, and what to do.
            if (targetFound) {

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

                }

                // Apply desired axes motions to the drivetrain.
                ch.moveRobot(-drive, strafe, turn);
                sleep(10);
            }
            else {
                ch.moveRobot(0,0,0);
            }
        }
    }
}