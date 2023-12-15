package org.firstinspires.ftc.teamcode.setup;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class CH {

    public DcMotor backLDrive = null;
    public DcMotor frontLDrive = null;
    public DcMotor frontRDrive = null;
    public DcMotor backRDrive = null;

    public DcMotor winchMotor = null;
    public Servo hookArm = null;
    public Servo gate = null;

    public static final double armMIN_POS = 0.22;
    public static final double armPOS_1 = 0.52;
    public static final double armPOS_2 = 0.73;

    public static final double Tighten = -0.7;
    public static final double Loosen = 0.7;

    public boolean Front = true;

    public CH(HardwareMap hardwareMap){

     frontLDrive = hardwareMap.get(DcMotor.class, "frontL");
     backLDrive = hardwareMap.get(DcMotor.class, "backL");
     frontRDrive = hardwareMap.get(DcMotor.class, "frontR");
     backRDrive = hardwareMap.get(DcMotor.class, "backR");
     winchMotor = hardwareMap.get(DcMotor.class, "winch");
     hookArm = hardwareMap.get(Servo.class, "arm");
     gate = hardwareMap.get(Servo.class, "gate");

     frontLDrive.setDirection(DcMotor.Direction.REVERSE);
     backLDrive.setDirection(DcMotor.Direction.REVERSE);
     frontRDrive.setDirection(DcMotor.Direction.FORWARD);
     backRDrive.setDirection(DcMotor.Direction.FORWARD);

     frontLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
     frontRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
     backLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
     backRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
}