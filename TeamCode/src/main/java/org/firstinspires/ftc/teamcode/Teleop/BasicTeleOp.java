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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Basic Tele-op", group="Linear OpMode")

public class BasicTeleOp extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor backLDrive = null;
    private DcMotor frontLDrive = null;
    private DcMotor frontRDrive = null;
    private DcMotor backRDrive = null;
    private DcMotor winchMotor = null;
    private Servo hookArm = null;
   // private Servo flyWheel1 = null;
   // private Servo flyWheel2 = null;
    private Servo launcher = null;
    static final double POS_1 =  0.35;     // Maximum rotational position
    static final double POS_2 = 0.5;
    static final double MIN_POS     =  0.0;
    double  position = (POS_2 - MIN_POS) / 2; // Start at halfway position

    @Override
    public void runOpMode() {

        frontLDrive  = hardwareMap.get(DcMotor.class, "frontL");
        backLDrive  = hardwareMap.get(DcMotor.class, "backL");
        frontRDrive = hardwareMap.get(DcMotor.class, "frontR");
        backRDrive = hardwareMap.get(DcMotor.class, "backR");
        winchMotor = hardwareMap.get(DcMotor.class, "winch");
        hookArm = hardwareMap.get(Servo.class, "arm");
        launcher = hardwareMap.get(Servo.class, "launcher");
        //flyWheel1 = hardwareMap.get(Servo.class,"flyWheel1");
        //flyWheel2 = hardwareMap.get(Servo.class, "flyWheel2");

        frontLDrive.setDirection(DcMotor.Direction.REVERSE);
        backLDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRDrive.setDirection(DcMotor.Direction.FORWARD);
        backRDrive.setDirection(DcMotor.Direction.FORWARD);
        //flyWheel1.setDirection(Servo.Direction.REVERSE);

        hookArm.setPosition(MIN_POS);
        launcher.setPosition(0);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            if (gamepad1.a) {
                position = POS_2;
                hookArm.setPosition(position);
            }
            if (gamepad1.x) {
                position = POS_1;
                hookArm.setPosition(position);
            }
            if (gamepad1.b) {
                position = MIN_POS;
                hookArm.setPosition(position);
            }
            if (gamepad1.left_bumper){
                winchMotor.setPower(0.5);
            }
            else if (gamepad1.right_bumper) {
                winchMotor.setPower(-0.5);
            }
            else {
                winchMotor.setPower(0);
            }
            if (gamepad1.y){
                launcher.setPosition(.5);
            }


            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            // Send calculated power to wheels
            frontLDrive.setPower(leftFrontPower);
            frontRDrive.setPower(rightFrontPower);
            backLDrive.setPower(leftBackPower);
            backRDrive.setPower(rightBackPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("launcher",launcher.getPosition());
            telemetry.update();
        }
    }}
