package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.setup.CH;

@TeleOp
(name="FieldCentricTest", group="Linear OpMode")
public class FieldCentricTest extends LinearOpMode {

    private CH ch = null;

    double wristTargetPos = 0;
    int armTargetPosition = 0;
    double rightPincerPos = Constants.RT.C_RIGHT_CLOSE;
    double leftPincerPos = Constants.RT.C_LEFT_CLOSE;

    @Override
    public void runOpMode() throws InterruptedException {
        ch = new CH(hardwareMap, this);
        ch.wrist.setPosition(0.35);


        waitForStart();

        while (opModeIsActive()) {

            double max;
            if (gamepad2.dpad_up){ // tighten
                ch.winchMotor.setPower(Constants.RT.WINCH_TIGHTEN);
            }
            else if (gamepad2.dpad_down) { // loosen winch
                ch.winchMotor.setPower(Constants.RT.WINCH_LOOSEN);
            }
            else if (gamepad2.dpad_left || gamepad2.dpad_right){ // hold winch
                ch.winchMotor.setPower(0.3);
            }
            else { //zero out winch
                ch.winchMotor.setPower(0);
            }

            if (gamepad2.a){ // launch drone
                ch.launcher.setPosition(0.7);
            }
            if (gamepad2.b){
                ch.launcher.setPosition(0.19);
            }

            if (gamepad2.left_bumper){ // open left pincer
                leftPincerPos = Constants.RT.C_LEFT_OPEN;
            }
            else { // close
                leftPincerPos = Constants.RT.C_LEFT_CLOSE;
            }
            if (gamepad2.right_bumper){ // open right pincer
                rightPincerPos = Constants.RT.C_RIGHT_OPEN;
            }
            else { // close
                rightPincerPos = Constants.RT.C_RIGHT_CLOSE;
            }

            if (gamepad2.left_trigger > 0) {
                wristTargetPos = wristTargetPos + 0.01;
            }
            else if (gamepad2.y){
                wristTargetPos = 0.32;
            }
            else {
                wristTargetPos = wristTargetPos - 0.01;
            }

            if (gamepad2.y){
                armTargetPosition = Constants.RT.ARM_UP;
            }
            if (gamepad2.left_stick_y < 0 ){ // arm down
                armTargetPosition = armTargetPosition +15;
            }
            else if (gamepad2.left_stick_y > 0 ){ // arm up
                armTargetPosition = armTargetPosition -15;
            }

            ch.shoulder.setTargetPosition(armTargetPosition);
            ch.shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ch.shoulder.setPower(0.6);

            ch.wrist.setPosition(wristTargetPos);
            ch.leftPincer.setPosition(leftPincerPos);
            ch.rightPincer.setPosition(rightPincerPos);

            if (wristTargetPos < Constants.RT.WRIST_UP){
                wristTargetPos = Constants.RT.WRIST_UP;
            }
            if (wristTargetPos > Constants.RT.WRIST_DOWN){
                wristTargetPos = Constants.RT.WRIST_DOWN;
            }
            if (armTargetPosition < Constants.RT.ARM_DOWN){
                armTargetPosition = Constants.RT.ARM_DOWN;
            }
            if (armTargetPosition > Constants.RT.ARM_MAX){
                armTargetPosition = Constants.RT.ARM_MAX;
            }

//            if (gamepad1.options) {
//                ch.imu.resetYaw();
//            }

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

            if (gamepad1.left_bumper) {
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
            telemetry.update();

        }
    }
}


