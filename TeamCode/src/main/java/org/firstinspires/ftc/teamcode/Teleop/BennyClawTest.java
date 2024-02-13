package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
@TeleOp
public class BennyClawTest extends LinearOpMode {


    // Define class members
    Servo servoRight;
    Servo servoLeft;
    double position = 1;
    static final double INCREMENT = 0.001;     // amount to slew servo each CYCLE_MS cycle
    static final int CYCLE_MS = 20;     // period of each cycle
    static final double MAX_POS = 0.9;     // Maximum rotational position
    static final double MIN_POS = 0.5;     // Minimum rotational position

    boolean rampUp = true;


    @Override
    public void runOpMode() {

        servoLeft = hardwareMap.get(Servo.class, "leftHand");
        servoRight = hardwareMap.get(Servo.class, "rightHand");
        // Wait for the start button
        telemetry.addData(">", "Press Start to scan Servo.");
        telemetry.update();


        servoLeft.setPosition(0.6);
        waitForStart();

        while (opModeIsActive()) {


            if (gamepad1.b) {

                // Keep stepping up until we hit the max value.

                servoLeft.setPosition(0.5);

                // rampUp = !rampUp;   // Switch ramp direction
            }


            if (gamepad1.a) {
                // Keep stepping down until we hit the min value.

                servoLeft.setPosition(0.9);
                //rampUp = !rampUp;  // Switch ramp direction
            }

        }
        // Display the current value
        telemetry.addData("Servo Position", "%5.2f", position);
        telemetry.addData(">", "Press Stop to end test.");
        telemetry.update();

        // Set the servo to the new position and pause;

        //     sleep(CYCLE_MS);
        //   idle();

        // Set the servo to the new position and pause;


        // Signal done;
        telemetry.addData(">", "Done");
        telemetry.update();
    }
}
