package org.firstinspires.ftc.teamcode.setup;


import static org.firstinspires.ftc.teamcode.CS.RT;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;


public class CH {
    public DcMotor backLDrive = null;
    public DcMotor frontLDrive = null;
    public DcMotor frontRDrive = null;
    public DcMotor backRDrive = null;
    public DcMotor winchMotor = null;
    public DcMotor arm = null;

    public Servo wrist = null;
    public Servo leftPincer;
    public Servo rightPincer;

    public Servo launcher;
    public IMU imu;
    public boolean Front = true;

    private LinearOpMode opMode_ref = null;

    public CH(HardwareMap hardwareMap, LinearOpMode op){

        opMode_ref = op;
        frontLDrive = hardwareMap.get(DcMotor.class, "frontL");
        backLDrive = hardwareMap.get(DcMotor.class, "backL");
        frontRDrive = hardwareMap.get(DcMotor.class, "frontR");
        backRDrive = hardwareMap.get(DcMotor.class, "backR");
        winchMotor = hardwareMap.get(DcMotor.class, "winch");
        launcher = hardwareMap.get(Servo.class, "launcher");
        leftPincer = hardwareMap.get(Servo.class, "leftPincer");
        rightPincer = hardwareMap.get(Servo.class, "rightPincer");
        arm =hardwareMap.get(DcMotor.class, "arm");
        wrist = hardwareMap.get(Servo.class, "wrist");

        frontLDrive.setDirection(DcMotor.Direction.REVERSE);
        backLDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRDrive.setDirection(DcMotor.Direction.FORWARD);
        backRDrive.setDirection(DcMotor.Direction.FORWARD);

        frontLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample OpMode

        imu.resetYaw();
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    public void encoderReset(){
        backRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        frontLDrive.setPower(leftFrontPower);
        frontRDrive.setPower(rightFrontPower);
        backLDrive.setPower(leftBackPower);
        backRDrive.setPower(rightBackPower);
    }

    public void imuMove(double powerLevel, double heading) { //heading positive left
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double turn, headingError;
        headingError    = heading - orientation.getYaw(AngleUnit.DEGREES);
        turn   = Range.clip(headingError * RT.A_TURN_GAIN, -RT.A_MAX_AUTO_TURN, RT.A_MAX_AUTO_TURN) ;
        if (powerLevel < 0) {
            turn = turn * -1;  // reverse turn if going backwards
        }
        moveRobot(powerLevel,0, turn);  // Added 'strafe' as a parameter
    }

    public void imuTurn(double heading) {
        YawPitchRollAngles orientation;
        double turn, headingError;

        orientation = imu.getRobotYawPitchRollAngles();
        headingError    = heading - orientation.getYaw(AngleUnit.DEGREES);

        while(Math.abs(headingError) > 5 && opMode_ref.opModeIsActive()) {  // just guessing that heading error of 3 is close enough

            orientation = imu.getRobotYawPitchRollAngles();
            headingError    = heading - orientation.getYaw(AngleUnit.DEGREES);
            turn   = Range.clip(headingError * RT.A_TURN_GAIN, -RT.A_MAX_AUTO_TURN, RT.A_MAX_AUTO_TURN) ;
            moveRobot(0, 0, turn);
            opMode_ref.sleep(10);

        }
        moveRobot(0, 0, 0);  // stop motors when turn done
    }

    public void EncoderMove(int targetPosition) {

        double  power   = 0.1;
        boolean rampUp  = true;

        encoderReset();

        while (backRDrive.getCurrentPosition() < targetPosition && opMode_ref.opModeIsActive()){

            if (backRDrive.getCurrentPosition() > targetPosition*0.5 && rampUp) {
                rampUp = !rampUp;   // Switch ramp direction
            }

            if (rampUp) {
                // Keep stepping up until we hit the max value.
                power += RT.E_INCREMENT;
                if (power >= RT.E_MAX_POWER) {
                    power = RT.E_MAX_POWER;
                }
            }
            else {
                // Keep stepping down until we hit the min value.
                power -= RT.E_INCREMENT;
                if (power <= RT.E_MIN_POWER) {
                    power = RT.E_MIN_POWER;
                    // rampUp = !rampUp;  // Switch ramp direction
                }
            }

            moveRobot(power,0,0);

            opMode_ref.sleep(RT.E_CYCLE_MS);

        } // while
        moveRobot(0,0,0);
    }//public void

    public void moveAprilTag(VP vp){

        boolean targetNotReached = true;
        AprilTagDetection desiredTag = null;

        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)

        desiredTag = null;
        while (targetNotReached && opMode_ref.opModeIsActive()) {
            targetFound = false;
            List<AprilTagDetection> currentDetections = vp.aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    if ((vp.DESIRED_TAG_ID < 0) || (detection.id == vp.DESIRED_TAG_ID)) {                     //  Check to see if we want to track towards this tag.
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

                double rangeError = (desiredTag.ftcPose.range - RT.A_DESIRED_DISTANCE);
                double headingError = desiredTag.ftcPose.bearing;
                double yawError = -desiredTag.ftcPose.yaw;

                if ((rangeError < 4) && (Math.abs(headingError) < 6) && (Math.abs(yawError) < 6)) {
                    drive = 0;
                    turn = 0;
                    strafe = 0;
                    targetNotReached = false;
                } else {
                    drive = Range.clip(rangeError * RT.A_SPEED_GAIN, -RT.A_MAX_AUTO_SPEED, RT.A_MAX_AUTO_SPEED);
                    turn = Range.clip(headingError * RT.A_TURN_GAIN, -RT.A_MAX_AUTO_TURN, RT.A_MAX_AUTO_TURN);
                    strafe = Range.clip(-yawError * RT.A_STRAFE_GAIN, -RT.A_MAX_AUTO_STRAFE, RT.A_MAX_AUTO_STRAFE);

                }

                // Apply desired axes motions to the drivetrain.
                moveRobot(-drive, strafe, turn);
                opMode_ref.sleep(10);
            }
            else {
                moveRobot(0,0,0);
            }
        }
    }

}