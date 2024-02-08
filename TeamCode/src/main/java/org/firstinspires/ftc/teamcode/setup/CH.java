package org.firstinspires.ftc.teamcode.setup;

import static android.os.SystemClock.sleep;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.bosch.BNO055IMU;
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

    public static final double armMIN_POS = 0.25;
    public static final double armPOS_1 = 0.52;
    public static final double armPOS_2 = 0.65;

    public static final double Tighten = -0.7;
    public static final double Loosen = 0.7;
    public IMU imu;
    public boolean Front = true;

    public static final double E_INCREMENT = 0.01;     // amount to ramp motor each CYCLE_MS cycle
    public static final int    E_CYCLE_MS = 25;     // period of each cycle
    public static final double E_MAX_POWER = 0.4;     // Maximum FWD power applied to motor
    public static final double E_MIN_POWER = 0.1;     // Maximum REV power applied to motor


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

     backRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

    public void imuReset(){
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
        turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
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
        while(Math.abs(headingError) > 20) {  // just guessing that heading error of 3 is close enough
            turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
            moveRobot(0, 0, turn);
            sleep(10);

        }
        moveRobot(0, 0, 0);  // stop motors when turn done
    }

    public void EncoderMove(int targetPosition) {

        double  power   = 0.1;
        boolean rampUp  = true;

        imuReset();

        while (backRDrive.getCurrentPosition() < targetPosition){

            if (backRDrive.getCurrentPosition() > targetPosition*0.5 && rampUp) {
                rampUp = !rampUp;   // Switch ramp direction
            }

            if (rampUp) {
                // Keep stepping up until we hit the max value.
                power += E_INCREMENT;
                if (power >= E_MAX_POWER) {
                    power = E_MAX_POWER;
                }
            }
            else {
                // Keep stepping down until we hit the min value.
                power -= E_INCREMENT;
                if (power <= E_MIN_POWER) {
                    power = E_MIN_POWER;
                    // rampUp = !rampUp;  // Switch ramp direction
                }
            }

            moveRobot(power,0,0);

            sleep(E_CYCLE_MS);

        } // while
        moveRobot(0,0,0);
    }//public void

}