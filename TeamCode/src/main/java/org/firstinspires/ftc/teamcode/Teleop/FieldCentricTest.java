package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.CaptainHook.Constants.CS;
import org.firstinspires.ftc.teamcode.CaptainHook.CH;

@TeleOp
(name="FieldCentricTest", group="Linear OpMode")
public class FieldCentricTest extends LinearOpMode {

    private CH ch = null;

    int armExtTargetPos = 0;
    double wristTargetPos = CS.WRIST_UP;
    int shoulderTargetPos = 0;
    double rightPincerPos = CS.C_RIGHT_CLOSE;
    double leftPincerPos = CS.C_LEFT_CLOSE;
    private ElapsedTime pickupTime = new ElapsedTime();

    boolean autoPickup = false;
    boolean rightTriggerPressed = false;
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();

    public boolean dropPixelCalled = false;
    @Override
    public void runOpMode() {
        ch = new CH(hardwareMap, this);
        ch.wrist.setPosition(CS.WRIST_UP);
        ch.rightPincer.setPosition(CS.C_RIGHT_CLOSE);
        ch.leftPincer.setPosition(CS.C_LEFT_CLOSE);

        ch.armEncoderReset();

        ch.backLDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ch.backRDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        boolean launchToggle = true;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {

            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            WinchControl();

            DroneLauncherControl();

            armSetPositions();

            ArmMove();

            AutoPickup();

            PincerControl();

            WristControl();

            maxPos();

            if (gamepad1.back) {
                ch.imu.resetYaw();
            }

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double botHeading = ch.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            if (gamepad1.left_trigger > 0 || gamepad1.right_trigger > 0) {
                ch.frontLDrive.setPower(0.3 * frontLeftPower);
                ch.frontRDrive.setPower(0.3 * frontRightPower);
                ch.backLDrive.setPower(0.3 * backLeftPower);
                ch.backRDrive.setPower(0.3 * backRightPower);
            }
            else { //slow button
                ch.frontLDrive.setPower(frontLeftPower);
                ch.frontRDrive.setPower(frontRightPower);
                ch.backLDrive.setPower(backLeftPower);
                ch.backRDrive.setPower(backRightPower);
            }

            telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);
            telemetry.addData("Launcher Position", ch.launcher.getPosition());
            telemetry.addData("arm position", ch.shoulder.getCurrentPosition());
            telemetry.addData("right pincer", ch.rightPincer.getPosition());
            telemetry.addData("left pincer", ch.leftPincer.getPosition());
            telemetry.addData("left pincer", pickupTime.milliseconds());
            telemetry.update();


        }
    }
    public void DroneLauncherControl(){
        if (gamepad2.right_trigger > 0 ) { //toggle drone launch and close
            ch.launcher.setPosition(0.7);
            armExtTargetPos = 1040;
            shoulderTargetPos = 1235;
        }
    }
    public void AutoPickup(){
        if (currentGamepad1.right_trigger > 0 && previousGamepad1.right_trigger == 0){
            wristTargetPos = CS.WRIST_DOWN;
            rightPincerPos = CS.C_RIGHT_OPEN;
            leftPincerPos = CS.C_LEFT_OPEN;
            rightTriggerPressed = true;
        }
        if (currentGamepad1.right_trigger == 0 && previousGamepad1.right_trigger > 0){
            autoPickup = true;
            pickupTime.reset();
        }

        if (autoPickup) {
            leftPincerPos = CS.C_LEFT_CLOSE;
            rightPincerPos = CS.C_RIGHT_CLOSE;
            if (pickupTime.milliseconds() >  300) {
                autoPickup = false;
                rightTriggerPressed = false;
            }
        }
    }
    public void WinchControl(){
        if (gamepad2.dpad_up)  // tighten
            ch.winchMotor.setPower(CS.WINCH_TIGHTEN);
        else if (gamepad2.dpad_down)  // loosen winch
            ch.winchMotor.setPower(CS.WINCH_LOOSEN);
        else if (gamepad2.dpad_left || gamepad2.dpad_right)  // hold winch
            ch.winchMotor.setPower(0.2);
        else  //zero out winch
            ch.winchMotor.setPower(0);

    }
    public void armSetPositions(){
        if (gamepad2.x) {
            shoulderTargetPos = 650;
            armExtTargetPos = 300;
        } else if (gamepad2.y) {
            shoulderTargetPos = 700;
            armExtTargetPos = 1000 ;
        } else if (gamepad2.b) {
            shoulderTargetPos = 0;
            armExtTargetPos = 0;
        } else if (gamepad2.a) {
            shoulderTargetPos = 420;
            armExtTargetPos = 0;
        }
    }
    public void WristControl(){
        if (gamepad2.left_trigger > 0) {
            wristTargetPos = CS.WRIST_DOWN;
        } else if (rightTriggerPressed == true) {
            telemetry.addData("working", true);
        }
        else if (ch.shoulder.getCurrentPosition() < 300) {
            if (wristTargetPos != CS.WRIST_UP)
                wristTargetPos = CS.WRIST_UP;
        } else if (ch.shoulder.getCurrentPosition() < 600) {
            if (wristTargetPos != 0.205)
                wristTargetPos = 0.205;
        } else {
            if (wristTargetPos != 0.15)
                wristTargetPos = 0.15;
        }
    }

    public void maxPos(){
        if (ch.armExtender.getCurrentPosition() > 40){ // if arm extender is out dont put arm down all the way
            if (shoulderTargetPos < CS.ARM_DOWN_EXT)
                shoulderTargetPos = CS.ARM_DOWN_EXT;
        }
        else { // if arm extender is in put arm down all the way
            if (shoulderTargetPos < CS.ARM_DOWN)
                shoulderTargetPos = CS.ARM_DOWN;
        }

        if (shoulderTargetPos > CS.ARM_MAX ) //max shoulder
            shoulderTargetPos = CS.ARM_MAX ;

        if (armExtTargetPos < CS.EXT_MIN ) //min extension
            armExtTargetPos = CS.EXT_MIN;

        if (armExtTargetPos > CS.EXT_MAX ) //max extension
            armExtTargetPos = CS.EXT_MAX;

        ch.shoulder.setTargetPosition(shoulderTargetPos);
        ch.shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ch.shoulder.setPower(0.6);

        ch.armExtender.setTargetPosition(armExtTargetPos);
        ch.armExtender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ch.armExtender.setPower(1);

        ch.wrist.setPosition(wristTargetPos);
        ch.leftPincer.setPosition(leftPincerPos);
        ch.rightPincer.setPosition(rightPincerPos);
    }
    public void PincerControl(){
        if (gamepad2.left_bumper) // open left pincer
            leftPincerPos = CS.C_LEFT_OPEN;
        else if (rightTriggerPressed);
        else // close
            leftPincerPos = CS.C_LEFT_CLOSE;

        if (gamepad2.right_bumper) // open right pincer
            rightPincerPos = CS.C_RIGHT_OPEN;
        else if (rightTriggerPressed);
        else // close
            rightPincerPos = CS.C_RIGHT_CLOSE;

    }
    public void ArmMove(){
        if (gamepad2.left_stick_y < 0) // arm down
            shoulderTargetPos = shoulderTargetPos + 15;
        else if (gamepad2.left_stick_y > 0) // arm up
            shoulderTargetPos = shoulderTargetPos - 15;

        if (gamepad2.right_stick_y < 0) //extender control
            armExtTargetPos = armExtTargetPos + 15;
        if (gamepad2.right_stick_y > 0)
            armExtTargetPos = armExtTargetPos - 15;
    }
}



