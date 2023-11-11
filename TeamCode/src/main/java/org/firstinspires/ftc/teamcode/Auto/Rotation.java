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

package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.CH;


@TeleOp(name="autonomous", group="Linear OpMode")

public class Rotation extends LinearOpMode {

    private CH ch = null;
    private boolean gamepadPressed = false;
    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() {

        ch = new CH(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Gamepad A is front Gamepad B is back", ch.Front);
        telemetry.update();
        while (gamepadPressed == false) {
            if (gamepad1.a) {
                ch.Front = true;
                gamepadPressed = true;

            }
            if (gamepad1.b) {
                ch.Front = false;
                gamepadPressed = true;
            }
        }
        telemetry.addData("Front =", ch.Front);
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (ch.Front == true) {
                ch.moveRobot(0.5, -0.3, 0);
                sleep(1000);
                ch.moveRobot(0, 0, 0);
                ch.moveRobot(-0.5, 0, 0);
                sleep(800);
                ch.moveRobot(0,0,0);
            }
            else {
                ch.moveRobot(0.55,0.2,0);
                sleep(1000);
                ch.moveRobot(0,0,0);
                sleep(100);
                ch.moveRobot(-0.5,0,0);
                sleep(200);
                ch.moveRobot(0,0,-0.6);
                sleep(500);
                ch.moveRobot(0,0,0);
            }

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
        }
    }


}
