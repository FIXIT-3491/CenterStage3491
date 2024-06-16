package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CaptainHook.Constants;

@Autonomous(name = "IntakeTest", group = "Linear OpMode")
public class IntakeTest extends LinearOpMode {

    // Define class members
    public Servo leftPincer;
    public Servo rightPincer;

    public CRServo spinnerIntake;

    public TouchSensor rightIntake;
    public TouchSensor leftIntake;

    public ElapsedTime intakeTimer = new ElapsedTime();

    public ElapsedTime spinTimer = new ElapsedTime();




    @Override
    public void runOpMode() {

         rightPincer = hardwareMap.get(Servo.class, "rightPincer");
          leftPincer = hardwareMap.get(Servo.class, "leftPincer");
        spinnerIntake = hardwareMap.get(CRServo.class, "intake");
        leftIntake = hardwareMap.get(TouchSensor.class, "leftIntake");
        rightIntake = hardwareMap.get(TouchSensor.class, "rightIntake");


        leftPincer.setPosition(Constants.CS.C_LEFT_OPEN);
        rightPincer.setPosition(Constants.CS.C_RIGHT_OPEN);

        waitForStart();
        while (opModeIsActive()) {

            // Check if either of the intake sensors are not pressed

            if (leftIntake.isPressed()) {
                leftPincer.setPosition(Constants.CS.C_LEFT_CLOSE);
            } else {
                leftPincer.setPosition(Constants.CS.C_LEFT_OPEN);
            }
            if (rightIntake.isPressed()) {
                rightPincer.setPosition(Constants.CS.C_RIGHT_CLOSE);
            } else {
                rightPincer.setPosition(Constants.CS.C_RIGHT_OPEN);
            }


            if (leftIntake.isPressed() && rightIntake.isPressed()) {

                // Stop the spinner intake

                spinnerIntake.setPower(0);
//
                // Reset the intake timer
                intakeTimer.reset();
            }
            else {
                // If the timer has been running for less than 2000 milliseconds (2 seconds)
                if (intakeTimer.milliseconds() < 2000) {
                    // Do nothing, effectively waiting for the 2-second delay to pass
                    // You can add a comment here to indicate the waiting period
                } else {
                    // If 2 seconds have passed, set the spinner intake power to -1
                    spinnerIntake.setPower(-1);
                }
            }

            telemetry.addData(" left Servo Position", "%5.2f", leftIntake.getValue());
            telemetry.addData("right Servo Position", "%5.2f", rightIntake.getValue());
            telemetry.addData(">", "Press Stop to end test.");
            telemetry.update();

        }


        //do the intake!!!


    }

}














































































































































































//Ryan was here hehehe