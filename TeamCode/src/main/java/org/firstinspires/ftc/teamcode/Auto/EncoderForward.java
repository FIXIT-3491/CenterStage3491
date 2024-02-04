package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.setup.CH;

@Autonomous(name="EncoderForward", group="Linear OpMode")

public class EncoderForward extends LinearOpMode {

    private CH ch = null;
    private ElapsedTime stepTimer = new ElapsedTime();

    double  power   = 0.1;
    boolean rampUp  = true;

    @Override
    public void runOpMode() {

        ch = new CH(hardwareMap);
        telemetry.addData("Status","initialized ");
        telemetry.update();

        waitForStart();
        if (opModeIsActive()){

            EncoderMove(750);

        } // if active
    } // run op mode

    public void EncoderMove(int targetPosition) {

        ch.backRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ch.backRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (ch.backRDrive.getCurrentPosition() < targetPosition){

            if (ch.backRDrive.getCurrentPosition() > targetPosition*0.5 && rampUp) {
                rampUp = !rampUp;   // Switch ramp direction
            }

            if (rampUp) {
                // Keep stepping up until we hit the max value.
                power += ch.E_INCREMENT;
                if (power >= ch.E_MAX_POWER) {
                    power = ch.E_MAX_POWER;
                }
            }
            else {
                // Keep stepping down until we hit the min value.
                power -= ch.E_INCREMENT;
                if (power <= ch.E_MIN_POWER) {
                    power = ch.E_MIN_POWER;
                    // rampUp = !rampUp;  // Switch ramp direction
                }
            }

            ch.moveRobot(power,0,0);

            telemetry.addData("Encoder poz",ch.backRDrive.getCurrentPosition() );
            telemetry.addData("motor power",power);
            telemetry.update();

            sleep(ch.E_CYCLE_MS);

        } // while
        ch.moveRobot(0,0,0);
    }//public void

} //linear op mode
