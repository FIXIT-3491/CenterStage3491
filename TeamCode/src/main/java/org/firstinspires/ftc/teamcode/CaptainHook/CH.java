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
    public SparkFunOTOSConfig myOtos;

    SparkFunOTOSConfig.Pose2D pos;


    public boolean Front = true;

    private LinearOpMode opMode_ref = null;

    public CH(HardwareMap hardwareMap, LinearOpMode op){

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
    public void configureOtos() {


        myOtos.setLinearUnit(SparkFunOTOSConfig.LinearUnit.INCHES);
        myOtos.setAngularUnit(SparkFunOTOSConfig.AngularUnit.DEGREES);

        SparkFunOTOSConfig.Pose2D offset = new SparkFunOTOSConfig.Pose2D(3, -1, 90);
        myOtos.setOffset(offset);

        myOtos.setLinearScalar(1.055);
        myOtos.setAngularScalar(1.0);

        myOtos.calibrateImu();

        myOtos.resetTracking();

        SparkFunOTOSConfig.Pose2D currentPosition = new SparkFunOTOSConfig.Pose2D(0, 0, 0);
        myOtos.setPosition(currentPosition);

        SparkFunOTOSConfig.Version hwVersion = new SparkFunOTOSConfig.Version();
        SparkFunOTOSConfig.Version fwVersion = new SparkFunOTOSConfig.Version();
        myOtos.getVersionInfo(hwVersion, fwVersion);

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
    public void moveRobotSparkfun(double x, double y, double yaw) {

        // Calculate wheel powers.
        double leftFrontPower    =  x +y +yaw;
        double rightFrontPower   =  x -y -yaw;
        double leftBackPower     =  x -y +yaw;
        double rightBackPower    =  x +y -yaw;

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
        opMode_ref.sleep(10);
    }
    public void otosDrive(double targetX, double targetY, double targetHeading) {
        double drive, strafe, turn;
        double currentRange, targetRange, initialBearing, targetBearing, xError, yError, yawError;
        double opp, adj;

        SparkFunOTOSConfig.Pose2D currentPos = myPosition();
        xError = targetX-currentPos.x;
        yError = targetY-currentPos.y;
        yawError = targetHeading-currentPos.h;

        while(opMode_ref.opModeIsActive() && ((Math.abs(xError) > 0.5) || (Math.abs(yError) > 0.5)
                || (Math.abs(yawError) > 4)) ) {
            // Use the speed and turn "gains" to calculate how we want the robot to move.
            drive  = Range.clip(xError * CS.SPARKFUN_SPEED_GAIN, -CS.SPARKFUN_MAX_AUTO_SPEED, CS.SPARKFUN_MAX_AUTO_SPEED);
            strafe = Range.clip(yError * CS.SPARKFUN_STRAFE_GAIN, -CS.SPARKFUN_MAX_AUTO_STRAFE, CS.SPARKFUN_MAX_AUTO_STRAFE);
            turn   = Range.clip(yawError * CS.SPARKFUN_TURN_GAIN, -CS.SPARKFUN_MAX_AUTO_TURN, CS.SPARKFUN_MAX_AUTO_TURN) ;

            opMode_ref.telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            // current x,y swapped due to 90 degree rotation
            opMode_ref.telemetry.addData("current X coordinate", currentPos.x);
            opMode_ref.telemetry.addData("current Y coordinate", currentPos.y);
            opMode_ref.telemetry.addData("current Heading angle", currentPos.h);
            opMode_ref.telemetry.addData("target X coordinate", targetX);
            opMode_ref.telemetry.addData("target Y coordinate", targetY);
            opMode_ref.telemetry.addData("target Heading angle", targetHeading);
            opMode_ref.telemetry.addData("xError", xError);
            opMode_ref.telemetry.addData("yError", yError);
            opMode_ref.telemetry.addData("yawError", yawError);
            opMode_ref.telemetry.update();

            // Apply desired axes motions to the drivetrain.
            moveRobotSparkfun(drive, strafe, turn);

            // then recalc error
            currentPos = myPosition();
            xError = targetX-currentPos.x;
            yError = targetY-currentPos.y;
            yawError = targetHeading-currentPos.h;
        }
        moveRobotSparkfun(0,0,0);
        currentPos = myPosition();
        opMode_ref.telemetry.addData("current X coordinate", currentPos.x);
        opMode_ref.telemetry.addData("current Y coordinate", currentPos.y);
        opMode_ref.telemetry.addData("current Heading angle", currentPos.h);
        opMode_ref.telemetry.update();
    }
    SparkFunOTOSConfig.Pose2D myPosition() {
        pos = myOtos.getPosition();
        SparkFunOTOSConfig.Pose2D myPos = new SparkFunOTOSConfig.Pose2D(pos.y, pos.x, -pos.h);
        return(myPos);
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

}