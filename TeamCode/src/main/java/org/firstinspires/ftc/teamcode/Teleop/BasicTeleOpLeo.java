package org.firstinspires.ftc.teamcode.Teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.setup.CH;
import org.firstinspires.ftc.teamcode.setup.Constants.CS;

@TeleOp(name="Basic TeleOpLeo", group="Linear OpMode")

public class BasicTeleOpLeo extends LinearOpMode {

    private CH ch = null;

    int armExtTargetPos = 0;
    double wristTargetPos = CS.WRIST_UP;
    int shoulderTargetPos = 0;
    double rightPincerPos = CS.C_RIGHT_CLOSE;
    double leftPincerPos = CS.C_LEFT_CLOSE;

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
            double max;

            if (gamepad2.dpad_up)  // tighten
                ch.winchMotor.setPower(CS.WINCH_TIGHTEN);
            else if (gamepad2.dpad_down)  // loosen winch
                ch.winchMotor.setPower(CS.WINCH_LOOSEN);
            else if (gamepad2.dpad_left || gamepad2.dpad_right)  // hold winch
                ch.winchMotor.setPower(0.2);
            else  //zero out winch
                ch.winchMotor.setPower(0);

            if (gamepad2.right_trigger > 0 ) { //toggle drone launch and close
                if (launchToggle) {
                    ch.launcher.setPosition(0.7);
                    launchToggle = false;
                } else {
                    ch.launcher.setPosition(0.19);
                    launchToggle = true;
                }
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
                shoulderTargetPos = 480;
                armExtTargetPos = 10;
            } else {
                if (gamepad2.left_stick_y < 0) // arm down
                    shoulderTargetPos = shoulderTargetPos + 15;
                else if (gamepad2.left_stick_y > 0) // arm up
                    shoulderTargetPos = shoulderTargetPos - 15;
            }

            if (gamepad2.left_bumper) // open left pincer
                leftPincerPos = CS.C_LEFT_OPEN;
            else // close
                leftPincerPos = CS.C_LEFT_CLOSE;

            if (gamepad2.right_bumper) // open right pincer
                rightPincerPos = CS.C_RIGHT_OPEN;
            else // close
                rightPincerPos = CS.C_RIGHT_CLOSE;

            if (gamepad2.right_stick_y < 0) //extender control
                armExtTargetPos = armExtTargetPos + 10;
            else if (gamepad2.right_stick_y > 0)
                armExtTargetPos = armExtTargetPos - 10;

            if (gamepad2.left_trigger > 0 ) //wrist control
                wristTargetPos = CS.WRIST_DOWN;
            else
                wristTargetPos = wristTargetPos + 0.05;

                if (gamepad2.left_trigger > 0 ) { //wrist control
                } else if (ch.shoulder.getCurrentPosition() < 310) {
                    if (wristTargetPos > CS.WRIST_UP)
                        wristTargetPos = CS.WRIST_UP;
                } else if (ch.shoulder.getCurrentPosition() < 600) {
                    if (wristTargetPos > 0.205)
                        wristTargetPos = 0.205;
                } else {
                    if (wristTargetPos != 0.15)
                        wristTargetPos = 0.15;
                }
// wrist 0.2049 armExt 30 shoulder 355
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

                double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
                double lateral = gamepad1.left_stick_x;
                double yaw = gamepad1.right_stick_x;
                double leftFrontPower = axial + lateral + yaw;
                double rightFrontPower = axial - lateral - yaw;
                double leftBackPower = axial - lateral + yaw;
                double rightBackPower = axial + lateral - yaw;
                max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
                max = Math.max(max, Math.abs(leftBackPower));
                max = Math.max(max, Math.abs(rightBackPower));
                if (max > 1.0) {
                    leftFrontPower /= max;
                    rightFrontPower /= max;
                    leftBackPower /= max;
                    rightBackPower /= max;
                }
                if (gamepad1.left_bumper) { // slow power
                    ch.frontLDrive.setPower(0.3 * leftFrontPower);
                    ch.frontRDrive.setPower(0.3 * rightFrontPower);
                    ch.backLDrive.setPower(0.3 * leftBackPower);
                    ch.backRDrive.setPower(0.3 * rightBackPower);
                } else { // full power
                    ch.frontLDrive.setPower(0.8 *leftFrontPower);
                    ch.frontRDrive.setPower(0.8 *rightFrontPower);
                    ch.backLDrive.setPower(0.8 *leftBackPower);
                    ch.backRDrive.setPower(0.8 *rightBackPower);
                }

                telemetry.addData("Front left/Right", "%4.2f, %4.2f", ch.frontLDrive.getCurrent(CurrentUnit.AMPS), ch.frontRDrive.getCurrent(CurrentUnit.AMPS));
                telemetry.addData("Back left/Right", "%4.2f, %4.2f", ch.backLDrive.getCurrent(CurrentUnit.AMPS), ch.backRDrive.getCurrent(CurrentUnit.AMPS));
                telemetry.addData("Front left/Right Current", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
                telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
                telemetry.addData("Launcher Position", ch.launcher.getPosition());
                telemetry.addData("shoulder position", ch.shoulder.getCurrentPosition());
                telemetry.addData("armExt position", ch.armExtender.getCurrentPosition());
                telemetry.addData("right pincer", ch.rightPincer.getPosition());
                telemetry.addData("left pincer", ch.leftPincer.getPosition());
                telemetry.addData("wrist", ch.wrist.getPosition());
                telemetry.update();
            }

        }
    }
