/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.setup.CH;

import org.firstinspires.ftc.teamcode.CS.RT;



@TeleOp(name="Basic TeleOp", group="Linear OpMode")

public class BasicTeleOp extends LinearOpMode {

    private CH ch = null;
    private ElapsedTime runtime = new ElapsedTime();

    double wristTargetPos = 0.35;
    int armTargetPosition = 0;

    @Override
    public void runOpMode() {
        ch = new CH(hardwareMap, this);
        ch.launcher.setPosition(0.19);
        ch.wrist.setPosition(0.35);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            double max;

            if (gamepad2.dpad_up){ // tighten
                ch.winchMotor.setPower(RT.W_TIGHTEN);
            }
            else if (gamepad2.dpad_down) { // loosen winch
                ch.winchMotor.setPower(RT.W_LOOSEN);
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

            if (gamepad2.left_bumper){ // open left pincer
                ch.leftPincer.setPosition(0.2);
            }
            else { // close
                ch.leftPincer.setPosition(0.5);
            }

            if (gamepad2.right_bumper){ // open right pincer
                ch.rightPincer.setPosition(0.85);
            }
            else { // close
                ch.rightPincer.setPosition(0.55);
            }
            if (gamepad2.y){
                armTargetPosition = 1500;

            }
            if (gamepad2.left_stick_y > 0 ){ // arm up
                armTargetPosition = armTargetPosition -10;
            }
            if (gamepad2.left_stick_y < 0 ){ // arm down
                armTargetPosition = armTargetPosition +10;
            }

            if (gamepad2.right_trigger > 0){ // wrist
                wristTargetPos = wristTargetPos - 0.01;
            }
            if (gamepad2.left_trigger > 0) {
                wristTargetPos = wristTargetPos + 0.01;
            }

            ch.arm.setTargetPosition(armTargetPosition);
            ch.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ch.arm.setPower(0.5);

            ch.wrist.setPosition(wristTargetPos);

            if (wristTargetPos < 0){
                wristTargetPos = 0;
            }
            if (wristTargetPos > 0.35){
                wristTargetPos = 0.35;
            }
            if (armTargetPosition < 30){
                armTargetPosition = 31;
            }
            if (armTargetPosition > 2150){
                armTargetPosition = 2149;
            }

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
            if (gamepad1.left_bumper) { //slow button
                ch.frontLDrive.setPower(0.3*leftFrontPower);
                ch.frontRDrive.setPower(0.3*rightFrontPower);
                ch.backLDrive.setPower(0.3*leftBackPower);
                ch.backRDrive.setPower(0.3*rightBackPower);
            } else{
                ch.frontLDrive.setPower(0.6 * leftFrontPower);
                ch.frontRDrive.setPower(0.6 * rightFrontPower);
                ch.backLDrive.setPower(0.6 * leftBackPower);
                ch.backRDrive.setPower(0.6 * rightBackPower);
            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Launcher Position", ch.launcher.getPosition());
            telemetry.addData("arm position", ch.arm.getCurrentPosition());
            telemetry.addData("Wrist position", ch.wrist.getPosition());
            telemetry.update();
        }

    }
}
