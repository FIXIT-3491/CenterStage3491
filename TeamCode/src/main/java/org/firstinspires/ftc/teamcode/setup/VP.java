package org.firstinspires.ftc.teamcode.setup;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
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

    private String CUP_POS = "Middle";
    public static final String TFOD_MODEL_ASSET = "rookDetection.tflite";
    public static final String[] LABELS = {"rook"};
    public TfodProcessor tfod;
    public boolean cupFound = false;

    public VP(HardwareMap hardwareMap) {
        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcam2 = hardwareMap.get(WebcamName.class, "Webcam 2");
    }

    public void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder()
                .setLensIntrinsics(1439.41944052, 1439.41944052, 970.51421863, 537.612825157)  //logitech c920
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



}

