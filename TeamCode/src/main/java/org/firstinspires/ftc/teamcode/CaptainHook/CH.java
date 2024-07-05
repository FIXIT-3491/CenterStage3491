package org.firstinspires.ftc.teamcode.CaptainHook;


import static org.firstinspires.ftc.teamcode.CaptainHook.Constants.CS;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;


public class CH {
    public DcMotorEx backLDrive = null;
    public DcMotorEx frontLDrive = null;
    public DcMotorEx frontRDrive = null;
    public DcMotorEx backRDrive = null;

    public DcMotor winchMotor = null;
    public DcMotor shoulder = null;
    public DcMotor armExtender = null;

    public Servo wrist = null;
    public Servo leftPincer;
    public Servo rightPincer;
    public Servo launcher;

    public CRServo spinnerIntake;


    public TouchSensor rightIntake;
    public TouchSensor leftIntake;
    public IMU imu;
    SparkFunOTOSConfig myOtos;


    public boolean Front = true;

    private LinearOpMode opMode_ref = null;

    public CH(HardwareMap hardwareMap, LinearOpMode op){

        configureOtos();

        opMode_ref  = op;
        frontLDrive = hardwareMap.get(DcMotorEx.class, "frontL");
        backLDrive  = hardwareMap.get(DcMotorEx.class, "backL");
        frontRDrive = hardwareMap.get(DcMotorEx.class, "frontR");
        backRDrive  = hardwareMap.get(DcMotorEx.class, "backR");

        winchMotor = hardwareMap.get(DcMotor.class, "winch");
        armExtender = hardwareMap.get(DcMotor.class, "armExtender");
        shoulder =   hardwareMap.get(DcMotor.class, "shoulder");


        leftPincer = hardwareMap.get(Servo.class, "leftPincer");
        rightPincer = hardwareMap.get(Servo.class, "rightPincer");
        wrist = hardwareMap.get(Servo.class, "wrist");
        launcher = hardwareMap.get(Servo.class, "launcher");

        spinnerIntake = hardwareMap.get(CRServo.class, "intake");

        leftIntake = hardwareMap.get(TouchSensor.class, "leftIntake");
        rightIntake = hardwareMap.get(TouchSensor.class, "rightIntake");
        myOtos = hardwareMap.get(SparkFunOTOSConfig.class, "sensor_otos");


        frontLDrive.setDirection(DcMotor.Direction.REVERSE);
        backLDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRDrive.setDirection(DcMotor.Direction.FORWARD);
        backRDrive.setDirection(DcMotor.Direction.FORWARD);
        armExtender.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armExtender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armExtender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample OpMode
        imu.resetYaw();
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    public void driveEncoderReset(){
        backRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    public void armEncoderReset(){
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armExtender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        turn   = Range.clip(headingError * CS.A_TURN_GAIN, -CS.A_MAX_AUTO_TURN, CS.A_MAX_AUTO_TURN) ;
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
            turn   = Range.clip(headingError * CS.IMU_TURN_GAIN, -CS.IMU_MAX_AUTO_TURN, CS.IMU_MAX_AUTO_TURN);
            moveRobot(0, 0, turn);
            opMode_ref.sleep(10);

        }
        moveRobot(0, 0, 0);  // stop motors when turn done
    }

    public void armMove(int targetPosition) {

        double power = 0.5;
        shoulder.setTargetPosition(targetPosition);

        shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        shoulder.setPower(power);

        while (shoulder.isBusy() && opMode_ref.opModeIsActive()) {
            opMode_ref.telemetry.addData("arm poz", shoulder.getCurrentPosition());
            opMode_ref.telemetry.update();
        }
        shoulder.setPower(0);
    }
    public void EncoderMove(int targetPosition) {

        double  power   = 0.6;
        boolean rampUp  = true;

        driveEncoderReset();

//        if (backRDrive.getCurrentPosition() > targetPosition*0.5 && rampUp) {
//            rampUp = !rampUp;   // Switch ramp direction
//        }
        while ((backRDrive.getCurrentPosition() + backLDrive.getCurrentPosition()) / 2 < Math.abs(targetPosition) && opMode_ref.opModeIsActive()){

            if (Math.abs((backRDrive.getCurrentPosition() + backLDrive.getCurrentPosition()) / 2) > Math.abs(targetPosition)*0.5 && rampUp) {
                rampUp = !rampUp;   // Switch ramp direction
            }

            if (rampUp) {
                // Keep stepping up until we hit the max value.
                power += CS.E_INCREMENT;
                if (power >= CS.E_MAX_POWER) {
                    power = CS.E_MAX_POWER;
                }
            }
            else {
                // Keep stepping down until we hit the min value.
                power -= CS.E_INCREMENT;
                if (power <= CS.E_MIN_POWER) {
                    power = CS.E_MIN_POWER;
                    // rampUp = !rampUp;  // Switch ramp direction
                }
            }

            if (targetPosition < 0) {
                power = -power;
            }

            moveRobot(power,0,0);

            opMode_ref.sleep(CS.E_CYCLE_MS);
            opMode_ref.telemetry.addData("encoder poz R", backRDrive.getCurrentPosition());
            opMode_ref.telemetry.addData("encoder poz L", backLDrive.getCurrentPosition());
            opMode_ref.telemetry.update();
        } // while
        moveRobot(0,0,0);
    }//public void
    //Shoulder:2000 ArmEXT:430 wrist:0.56

    public void dropPixel2(){
        shoulder.setTargetPosition(1700);
        shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulder.setPower(0.5);

        armExtender.setTargetPosition(1290);
        armExtender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armExtender.setPower(0.9);

        opMode_ref.sleep( 1500);

        wrist.setPosition(0.56);
        opMode_ref.sleep( 2000);

        rightPincer.setPosition(CS.C_RIGHT_OPEN);
    }

    public void dropPixel1(){
        shoulder.setTargetPosition(425);
        shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulder.setPower(0.8);


    }
    public void WhitePixel(){
        shoulder.setTargetPosition(200); // 185 or //240
        shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulder.setPower(1);

        wrist.setPosition(0.075);
        leftPincer.setPosition(CS.C_LEFT_OPEN);
    }

    public void closeArmAuto(){
        shoulder.setTargetPosition(0);
        shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulder.setPower(0.5);
        armExtender.setTargetPosition(0);
        armExtender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armExtender.setPower(1);

        wrist.setPosition(CS.WRIST_UP);
        rightPincer.setPosition(CS.C_RIGHT_CLOSE);
    }


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

                double rangeError = (desiredTag.ftcPose.range - CS.A_DESIRED_DISTANCE);
                double headingError = desiredTag.ftcPose.bearing+10;
                double yawError = desiredTag.ftcPose.yaw;

                if ((rangeError < 4) && (Math.abs(headingError) < 6) && (Math.abs(yawError) < 6)) {
                    drive = 0;
                    turn = 0;
                    strafe = 0;
                    targetNotReached = false;
                } else {
                    drive = Range.clip(rangeError * CS.A_SPEED_GAIN, -CS.A_MAX_AUTO_SPEED, CS.A_MAX_AUTO_SPEED);
                    turn = Range.clip(headingError * CS.A_TURN_GAIN, -CS.A_MAX_AUTO_TURN, CS.A_MAX_AUTO_TURN);
                    strafe = Range.clip(-yawError * CS.A_STRAFE_GAIN, -CS.A_MAX_AUTO_STRAFE, CS.A_MAX_AUTO_STRAFE);

                }

                // Apply desired axes motions to the drivetrain.
                moveRobot(drive, strafe, turn);
                opMode_ref.sleep(10);
            }
            else {
                moveRobot(0,0,0);
            }
        }
    }

    public void moveAprilTag2(VP vp){

        ElapsedTime stepTimer = new ElapsedTime();
        boolean targetNotReached = true;
        AprilTagDetection desiredTag = null;

        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)

        desiredTag = null;
        while (targetNotReached && opMode_ref.opModeIsActive() && stepTimer.milliseconds() < 4000) {
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

                double rangeError = (desiredTag.ftcPose.range - 10);
                double headingError = desiredTag.ftcPose.bearing;
                double yawError = desiredTag.ftcPose.yaw;

                if ((rangeError < 4) && (Math.abs(headingError) < 6) && (Math.abs(yawError) < 6)) {
                    drive = 0;
                    turn = 0;
                    strafe = 0;
                    targetNotReached = false;
                } else {
                    drive = Range.clip(rangeError * CS.A_SPEED_GAIN, -CS.A_MAX_AUTO_SPEED, CS.A_MAX_AUTO_SPEED);
                    turn = Range.clip(headingError * CS.A_TURN_GAIN, -CS.A_MAX_AUTO_TURN, CS.A_MAX_AUTO_TURN);
                    strafe = Range.clip(-yawError * CS.A_STRAFE_GAIN, -CS.A_MAX_AUTO_STRAFE, CS.A_MAX_AUTO_STRAFE);

                }

                // Apply desired axes motions to the drivetrain.
                moveRobot(drive, strafe, turn);
                opMode_ref.sleep(10);
            }
            else {
                moveRobot(0,0,0);
            }
        }
    }

    public void configureOtos() {

        // Set the desired units for linear and angular measurements. Can be either
        // meters or inches for linear, and radians or degrees for angular. If not
        // set, the default is inches and degrees. Note that this setting is not
        // stored in the sensor, it's part of the library, so you need to set at the
        // start of all your programs.
        // myOtos.setLinearUnit(SparkFunOTOS.LinearUnit.METERS);
        myOtos.setLinearUnit(SparkFunOTOSConfig.LinearUnit.INCHES);
        myOtos.setAngularUnit(SparkFunOTOSConfig.AngularUnit.DEGREES);

        // Assuming you've mounted your sensor to a robot and it's not centered,
        // you can specify the offset for the sensor relative to the center of the
        // robot. The units default to inches and degrees, but if you want to use
        // different units, specify them before setting the offset! Note that as of
        // firmware version 1.0, these values will be lost after a power cycle, so
        // you will need to set them each time you power up the sensor. For example, if
        // the sensor is mounted 5 inches to the left (negative X) and 10 inches
        // forward (positive Y) of the center of the robot, and mounted 90 degrees
        // clockwise (negative rotation) from the robot's orientation, the offset
        // would be {-5, 10, -90}. These can be any value, even the angle can be
        // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
        SparkFunOTOSConfig.Pose2D offset = new SparkFunOTOSConfig.Pose2D(0, 0, 0);
        myOtos.setOffset(offset);

        // Here we can set the linear and angular scalars, which can compensate for
        // scaling issues with the sensor measurements. Note that as of firmware
        // version 1.0, these values will be lost after a power cycle, so you will
        // need to set them each time you power up the sensor. They can be any value
        // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
        // first set both scalars to 1.0, then calibrate the angular scalar, then
        // the linear scalar. To calibrate the angular scalar, spin the robot by
        // multiple rotations (eg. 10) to get a precise error, then set the scalar
        // to the inverse of the error. Remember that the angle wraps from -180 to
        // 180 degrees, so for example, if after 10 rotations counterclockwise
        // (positive rotation), the sensor reports -15 degrees, the required scalar
        // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
        // robot a known distance and measure the error; do this multiple times at
        // multiple speeds to get an average, then set the linear scalar to the
        // inverse of the error. For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        myOtos.setLinearScalar(1.0);
        myOtos.setAngularScalar(1.0);

        // The IMU on the OTOS includes a gyroscope and accelerometer, which could
        // have an offset. Note that as of firmware version 1.0, the calibration
        // will be lost after a power cycle; the OTOS performs a quick calibration
        // when it powers up, but it is recommended to perform a more thorough
        // calibration at the start of all your programs. Note that the sensor must
        // be completely stationary and flat during calibration! When calling
        // calibrateImu(), you can specify the number of samples to take and whether
        // to wait until the calibration is complete. If no parameters are provided,
        // it will take 255 samples and wait until done; each sample takes about
        // 2.4ms, so about 612ms total
        myOtos.calibrateImu();

        // Reset the tracking algorithm - this resets the position to the origin,
        // but can also be used to recover from some rare tracking errors
        myOtos.resetTracking();

        // After resetting the tracking, the OTOS will report that the robot is at
        // the origin. If your robot does not start at the origin, or you have
        // another source of location information (eg. vision odometry), you can set
        // the OTOS location to match and it will continue to track from there.
        SparkFunOTOSConfig.Pose2D currentPosition = new SparkFunOTOSConfig.Pose2D(0, 0, 0);
        myOtos.setPosition(currentPosition);

//        // Get the hardware and firmware version
//        SparkFunOTOSConfig.Version hwVersion = new SparkFunOTOSConfig.Version();
//        SparkFunOTOSConfig.Version fwVersion = new SparkFunOTOSConfig.Version();
//        myOtos.getVersionInfo(hwVersion, fwVersion);
//
//        opMode_ref.telemetry.addLine("OTOS configured! Press start to get position data!");
//        opMode_ref.telemetry.addLine();
//        opMode_ref.telemetry.addLine(String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
//        opMode_ref.telemetry.addLine(String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
//        opMode_ref.telemetry.update();
    }

}