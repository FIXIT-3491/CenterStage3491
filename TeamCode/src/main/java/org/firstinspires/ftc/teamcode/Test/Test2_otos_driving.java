package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.CaptainHook.CH;
import org.firstinspires.ftc.teamcode.CaptainHook.Constants.CS;
import org.firstinspires.ftc.teamcode.CaptainHook.SparkFunOTOSConfig;

@Autonomous(name="Test2_otos_driving", group="test")
public class Test2_otos_driving extends LinearOpMode {
    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.035  ;   // 0.02 Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.15;   // 0.015 Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.035  ;   // 0.01 Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.5;   //  Clip the turn speed to this max value (adjust for your robot)

    private ElapsedTime runtime = new ElapsedTime();
    private CH ch = null;



    SparkFunOTOSConfig.Pose2D pos;

    @Override
    public void runOpMode() {
        ch = new CH(hardwareMap, this);

        // Get a reference to the sensor

        configureOtos();
        sleep(1000);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        ch.wrist.setPosition(CS.WRIST_UP);
        //moveRobotTest();



        otosDrive(84, 0, 0);// left spike mark

        otosDrive(84, 56, 0);// left spike mark
        otosDrive(108, 56, 0);// left spike mark

        otosDrive(84, 56, 0);// left spike mark

        otosDrive(84, 0, 0);// left spike mark
        otosDrive(0, 0, 0);// left spike mark


//        otosDrive(48, 24, 0);// left spike mark
//
//        otosDrive(0, 24, 0);// left spike mark
//
//        otosDrive(0, 0, 0);// left spike mark

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            otosDrive(0,0,0);
        }
    }

    private void configureOtos() {
        telemetry.addLine("Configuring OTOS...");
        telemetry.update();

        ch.myOtos.setLinearUnit(SparkFunOTOSConfig.LinearUnit.INCHES);
        ch.myOtos.setAngularUnit(SparkFunOTOSConfig.AngularUnit.DEGREES);

        SparkFunOTOSConfig.Pose2D offset = new SparkFunOTOSConfig.Pose2D(3, -1, 90);
        ch.myOtos.setOffset(offset);

        ch.myOtos.setLinearScalar(1.071);
        ch.myOtos.setAngularScalar(1.0);

        ch.myOtos.calibrateImu();

        ch.myOtos.resetTracking();

        SparkFunOTOSConfig.Pose2D currentPosition = new SparkFunOTOSConfig.Pose2D(0, 0, 0);
        ch.myOtos.setPosition(currentPosition);

        SparkFunOTOSConfig.Version hwVersion = new SparkFunOTOSConfig.Version();
        SparkFunOTOSConfig.Version fwVersion = new SparkFunOTOSConfig.Version();
        ch.myOtos.getVersionInfo(hwVersion, fwVersion);

    }

    /**
     * Move robot to a designated X,Y position and heading
     */
    void otosDrive(double targetX, double targetY, double targetHeading) {
        double drive, strafe, turn;
        double currentRange, targetRange, initialBearing, targetBearing, xError, yError, yawError;
        double opp, adj;

        SparkFunOTOSConfig.Pose2D currentPos = myPosition();
        xError = targetX-currentPos.x;
        yError = targetY-currentPos.y;
        yawError = targetHeading-currentPos.h;

        while(opModeIsActive() && ((Math.abs(xError) > 1) || (Math.abs(yError) > 0.5)
                || (Math.abs(yawError) > 4)) ) {
            // Use the speed and turn "gains" to calculate how we want the robot to move.
            drive  = Range.clip(xError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            strafe = Range.clip(yError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
            turn   = Range.clip(yawError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;

            telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            // current x,y swapped due to 90 degree rotation
            telemetry.addData("current X coordinate", currentPos.x);
            telemetry.addData("current Y coordinate", currentPos.y);
            telemetry.addData("current Heading angle", currentPos.h);
            telemetry.addData("target X coordinate", targetX);
            telemetry.addData("target Y coordinate", targetY);
            telemetry.addData("target Heading angle", targetHeading);
            telemetry.addData("xError", xError);
            telemetry.addData("yError", yError);
            telemetry.addData("yawError", yawError);
            telemetry.update();

            // Apply desired axes motions to the drivetrain.
            moveRobot(drive, strafe, turn);

            // then recalc error
            currentPos = myPosition();
            xError = targetX-currentPos.x;
            yError = targetY-currentPos.y;
            yawError = targetHeading-currentPos.h;
        }
        moveRobot(0,0,0);
        currentPos = myPosition();
        telemetry.addData("current X coordinate", currentPos.x);
        telemetry.addData("current Y coordinate", currentPos.y);
        telemetry.addData("current Heading angle", currentPos.h);
        telemetry.update();
    }

    /* the reported OTOS values are based on sensor orientation, convert to robot centric
        by swapping x and y and changing the sign of the heading
        */

    SparkFunOTOSConfig.Pose2D myPosition() {
        pos = ch.myOtos.getPosition();
        SparkFunOTOSConfig.Pose2D myPos = new SparkFunOTOSConfig.Pose2D(pos.y, pos.x, -pos.h);
        return(myPos);
    }

    void moveRobot(double x, double y, double yaw) {

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
        ch.frontLDrive.setPower(leftFrontPower);
        ch.frontRDrive.setPower(rightFrontPower);
        ch.backLDrive.setPower(leftBackPower);
        ch.backRDrive.setPower(rightBackPower);
        sleep(10);
    }


}