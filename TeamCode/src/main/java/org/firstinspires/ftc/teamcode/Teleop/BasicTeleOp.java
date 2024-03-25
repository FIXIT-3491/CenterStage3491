package org.firstinspires.ftc.teamcode.Teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.setup.CH;
import org.firstinspires.ftc.teamcode.Constants.RT;

@TeleOp(name="Basic TeleOp", group="Linear OpMode")

public class BasicTeleOp extends LinearOpMode {

    private CH ch = null;

    int armExtTargetPos = 0;
    double wristTargetPos = 0;
    int shoulderTargetPos = 0;
    double rightPincerPos = RT.C_RIGHT_CLOSE;
    double leftPincerPos = RT.C_LEFT_CLOSE;
    
    @Override
    public void runOpMode() {
        ch = new CH(hardwareMap, this);
        ch.wrist.setPosition(0.35);
        ch.armEncoderReset();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double max;
            if (gamepad2.dpad_up){ // tighten
                ch.winchMotor.setPower(RT.WINCH_TIGHTEN);
            }
            else if (gamepad2.dpad_down) { // loosen winch
                ch.winchMotor.setPower(RT.WINCH_LOOSEN);
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
                leftPincerPos = RT.C_LEFT_OPEN;
            }
            else { // close
                leftPincerPos = RT.C_LEFT_CLOSE;
            }

            if (gamepad2.right_bumper){ // open right pincer
                rightPincerPos = RT.C_RIGHT_OPEN;
            }
            else { // close
                rightPincerPos = RT.C_RIGHT_CLOSE;
            }

            if (gamepad2.left_trigger > 0) {
                wristTargetPos = wristTargetPos - 0.01;
            }
            else if (gamepad2.y){
                wristTargetPos = 0.32;
            }
            else {
                wristTargetPos = wristTargetPos + 0.01;
            }

            if (gamepad2.y){
                shoulderTargetPos = RT.ARM_UP;
            }
            if (gamepad2.left_stick_y < 0 ){ // arm down
                shoulderTargetPos = shoulderTargetPos +15;
            }
            else if (gamepad2.left_stick_y > 0 ){ // arm up
                shoulderTargetPos = shoulderTargetPos -15;
            }

            ch.shoulder.setTargetPosition(shoulderTargetPos);
            ch.shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ch.shoulder.setPower(0.6);


            ch.shoulder.setTargetPosition(shoulderTargetPos);
            ch.shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ch.shoulder.setPower(0.6);

           // ch.armExtender.setTargetPosition();

            ch.wrist.setPosition(wristTargetPos);
            ch.leftPincer.setPosition(leftPincerPos);
            ch.rightPincer.setPosition(rightPincerPos);

            if (wristTargetPos < RT.WRIST_UP){
                wristTargetPos = RT.WRIST_UP;
            }
            if (wristTargetPos > RT.WRIST_DOWN){
                wristTargetPos = RT.WRIST_DOWN;
            }
            if (shoulderTargetPos < RT.ARM_DOWN){
                shoulderTargetPos = RT.ARM_DOWN;
            }
            if (shoulderTargetPos > RT.ARM_MAX){
                shoulderTargetPos = RT.ARM_MAX;
            }

            ch.armExtender.setPower(gamepad2.right_stick_y);

            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));
            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }
            if (gamepad1.left_bumper) {
                ch.frontLDrive.setPower(0.3 * leftFrontPower);
                ch.frontRDrive.setPower(0.3 * rightFrontPower);
                ch.backLDrive.setPower(0.3 * leftBackPower);
                ch.backRDrive.setPower(0.3 * rightBackPower);
            }
            else { //slow button
                ch.frontLDrive.setPower(leftFrontPower);
                ch.frontRDrive.setPower(rightFrontPower);
                ch.backLDrive.setPower(leftBackPower);
                ch.backRDrive.setPower(rightBackPower);
            }

            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Launcher Position", ch.launcher.getPosition());
            telemetry.addData("arm position", ch.shoulder.getCurrentPosition());
            telemetry.update();
        }

    }
}
