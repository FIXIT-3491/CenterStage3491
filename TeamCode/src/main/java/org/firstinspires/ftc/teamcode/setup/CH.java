package org.firstinspires.ftc.teamcode.setup;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


public class CH {

    public double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    public double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    public double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    public double MAX_AUTO_SPEED = 0.2;   //  Clip the approach speed to this max value (adjust for your robot)
    public double MAX_AUTO_STRAFE= 0.2;   //  Clip the approach speed to this max value (adjust for your robot)
    public double MAX_AUTO_TURN  = 0.2;   //  Clip the turn speed to this max value (adjust for your robot)

    public DcMotor backLDrive = null;
    public DcMotor frontLDrive = null;
    public DcMotor frontRDrive = null;
    public DcMotor backRDrive = null;

    public DcMotor winchMotor = null;
    public Servo hookArm = null;
    public Servo gate = null;
    public Servo launcher = null;

    public static final double armMIN_POS = 0.22;
    public static final double armPOS_1 = 0.52;
    public static final double armPOS_2 = 0.73;

    public static final double Tighten = -0.7;
    public static final double Loosen = 0.7;
    public IMU imu;
    public boolean Front = true;

    public CH(HardwareMap hardwareMap){

     frontLDrive = hardwareMap.get(DcMotor.class, "frontL");
     backLDrive = hardwareMap.get(DcMotor.class, "backL");
     frontRDrive = hardwareMap.get(DcMotor.class, "frontR");
     backRDrive = hardwareMap.get(DcMotor.class, "backR");
     winchMotor = hardwareMap.get(DcMotor.class, "winch");
     hookArm = hardwareMap.get(Servo.class, "arm");
     gate = hardwareMap.get(Servo.class, "gate");
     launcher = hardwareMap.get(Servo.class, "launcher");


     frontLDrive.setDirection(DcMotor.Direction.REVERSE);
     backLDrive.setDirection(DcMotor.Direction.REVERSE);
     frontRDrive.setDirection(DcMotor.Direction.FORWARD);
     backRDrive.setDirection(DcMotor.Direction.FORWARD);

     frontLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
     frontRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
     backLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
     backRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

     imu = hardwareMap.get(IMU.class, "imu");
     RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
     RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;
     RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

     imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    public void imuInit(){


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
    public void imuMove(double powerLevel, double heading, double strafe) { //heading positive left
        YawPitchRollAngles orientation;
        double turn, headingError;

        orientation = imu.getRobotYawPitchRollAngles();
        headingError    = heading - orientation.getYaw(AngleUnit.DEGREES);
        turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
        if (powerLevel < 0) {
            turn = turn * -1;  // reverse turn if going backwards
        }
        moveRobot(powerLevel, strafe, turn);  // Added 'strafe' as a parameter
    }


}