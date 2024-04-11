package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.CaptainHook.Constants;
import org.firstinspires.ftc.teamcode.CaptainHook.CH;

@TeleOp
(name="FieldCentricTest", group="Linear OpMode")
public class FieldCentricTest extends LinearOpMode {

    private CH ch = null;

    int armExtTargetPos = 0;
    double wristTargetPos = Constants.CS.WRIST_UP;
    int shoulderTargetPos = 0;
    double rightPincerPos = Constants.CS.C_RIGHT_CLOSE;
    double leftPincerPos = Constants.CS.C_LEFT_CLOSE;
    private ElapsedTime stepTimer = new ElapsedTime();

    public boolean dropPixelCalled = false;
    @Override
    public void runOpMode() {
        ch = new CH(hardwareMap, this);
        ch.wrist.setPosition(Constants.CS.WRIST_UP);
        ch.rightPincer.setPosition(Constants.CS.C_RIGHT_CLOSE);
        ch.leftPincer.setPosition(Constants.CS.C_LEFT_CLOSE);

        ch.armEncoderReset();

        ch.backLDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ch.backRDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        boolean launchToggle = true;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            double max;

            if (gamepad2.dpad_up)  // tighten
                ch.winchMotor.setPower(Constants.CS.WINCH_TIGHTEN);
            else if (gamepad2.dpad_down)  // loosen winch
                ch.winchMotor.setPower(Constants.CS.WINCH_LOOSEN);
            else if (gamepad2.dpad_left || gamepad2.dpad_right)  // hold winch
                ch.winchMotor.setPower(0.2);
            else  //zero out winch
                ch.winchMotor.setPower(0);

            if (gamepad2.right_trigger > 0 ) { //toggle drone launch and close
                       ch.launcher.setPosition(0.7);
                       armExtTargetPos = 1040;
                       shoulderTargetPos = 1235;
                }

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
            } else {
                if (gamepad2.left_stick_y < 0) // arm down
                    shoulderTargetPos = shoulderTargetPos + 15;
                else if (gamepad2.left_stick_y > 0) // arm up
                    shoulderTargetPos = shoulderTargetPos - 15;
            }

            if (gamepad2.left_bumper || gamepad1.right_trigger > 0) // open left pincer
                leftPincerPos = Constants.CS.C_LEFT_OPEN;
            else // close
                leftPincerPos = Constants.CS.C_LEFT_CLOSE;

            if (gamepad2.right_bumper || gamepad1.right_trigger > 0) // open right pincer
                rightPincerPos = Constants.CS.C_RIGHT_OPEN;
            else // close
                rightPincerPos = Constants.CS.C_RIGHT_CLOSE;

            if (gamepad2.right_stick_y < 0) //extender control
                armExtTargetPos = armExtTargetPos + 15;
            else if (gamepad2.right_stick_y > 0)
                armExtTargetPos = armExtTargetPos - 15;

            if (gamepad2.left_trigger > 0  || gamepad1.right_trigger > 0 ) //wrist control
                wristTargetPos = Constants.CS.WRIST_DOWN;
            else
                wristTargetPos = wristTargetPos + 0.05;

            if (ch.rightPincer.getPosition() != Constants.CS.C_RIGHT_CLOSE && ch.leftPincer.getPosition() != Constants.CS.C_LEFT_CLOSE){
                stepTimer.reset();
            }

            if (gamepad1.left_trigger > 0 || gamepad1.right_trigger > 0) { //wrist control
            } else if (stepTimer.milliseconds() > 1000){
            } else if (ch.shoulder.getCurrentPosition() < 300) {
                if (wristTargetPos > Constants.CS.WRIST_UP)
                    wristTargetPos = Constants.CS.WRIST_UP;
            } else if (ch.shoulder.getCurrentPosition() < 600) {
                if (wristTargetPos > 0.205)
                    wristTargetPos = 0.205;
            } else {
                if (wristTargetPos != 0.15)
                    wristTargetPos = 0.15;
            }

            if (ch.armExtender.getCurrentPosition() > 40){ // if arm extender is out dont put arm down all the way
                if (shoulderTargetPos < Constants.CS.ARM_DOWN_EXT)
                    shoulderTargetPos = Constants.CS.ARM_DOWN_EXT;
            }
            else { // if arm extender is in put arm down all the way
                if (shoulderTargetPos < Constants.CS.ARM_DOWN)
                    shoulderTargetPos = Constants.CS.ARM_DOWN;
            }

            if (shoulderTargetPos > Constants.CS.ARM_MAX ) //max shoulder
                shoulderTargetPos = Constants.CS.ARM_MAX ;

            if (armExtTargetPos < Constants.CS.EXT_MIN ) //min extension
                armExtTargetPos = Constants.CS.EXT_MIN;

            if (armExtTargetPos > Constants.CS.EXT_MAX ) //max extension
                armExtTargetPos = Constants.CS.EXT_MAX;

            ch.shoulder.setTargetPosition(shoulderTargetPos);
            ch.shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ch.shoulder.setPower(0.6);

            ch.armExtender.setTargetPosition(armExtTargetPos);
            ch.armExtender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ch.armExtender.setPower(1);

            ch.wrist.setPosition(wristTargetPos);
            ch.leftPincer.setPosition(leftPincerPos);
            ch.rightPincer.setPosition(rightPincerPos);

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
            telemetry.addData("left pincer", stepTimer.milliseconds());
            telemetry.update();

        }
    }
}


