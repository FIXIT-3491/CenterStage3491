package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.setup.CH;


@Autonomous(name="TestCode", group="Linear OpMode")

public class TestCode extends LinearOpMode {

    private CH ch = null;
    private ElapsedTime stepTimer = new ElapsedTime();
    private int targetPosition = 1380;

    static final double INCREMENT   = 0.01;     // amount to ramp motor each CYCLE_MS cycle
    static final int    CYCLE_MS    = 25;     // period of each cycle
    static final double MAX_POWER   = 0.4;     // Maximum FWD power applied to motor
    static final double MIN_POWER   = 0.1;     // Maximum REV power applied to motor

    double  power   = 0.1;
    boolean rampUp  = true;

    @Override
    public void runOpMode() {

        ch = new CH(hardwareMap);
        telemetry.addData("Status","initialized ");
        telemetry.update();

        waitForStart();
        if (opModeIsActive()){


            while (ch.backRDrive.getCurrentPosition() < targetPosition){

                if (ch.backRDrive.getCurrentPosition() > targetPosition*0.5 && rampUp) {
                    rampUp = !rampUp;   // Switch ramp direction
                }

                if (rampUp) {
                    // Keep stepping up until we hit the max value.
                    power += INCREMENT ;
                    if (power >= MAX_POWER) {
                        power = MAX_POWER;
                    }
                }
                else {
                    // Keep stepping down until we hit the min value.
                    power -= INCREMENT ;
                    if (power <= MIN_POWER) {
                        power = MIN_POWER;
                        // rampUp = !rampUp;  // Switch ramp direction
                    }
                }

                ch.moveRobot(power,0,0);
                sleep(CYCLE_MS);

                telemetry.addData("Encoder poz",ch.backRDrive.getCurrentPosition() );
                telemetry.addData("motor power",power);
                telemetry.update();
            }
            ch.moveRobot(0,0,0);

        } // if active
    } // run op mode
} //linear op mode
