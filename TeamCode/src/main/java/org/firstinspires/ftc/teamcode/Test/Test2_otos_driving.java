package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.CaptainHook.CH;
import org.firstinspires.ftc.teamcode.CaptainHook.Constants.CS;
@Disabled
@Autonomous(name="Test2_otos_driving", group="test")
public class Test2_otos_driving extends LinearOpMode {
    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.

    private ElapsedTime runtime = new ElapsedTime();
    private CH ch = null;
    @Override
    public void runOpMode() {
        ch = new CH(hardwareMap, this);

        ch.configureOtos();
        sleep(1000);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        ch.wrist.setPosition(CS.WRIST_UP);
        ch.otosDrive(50,0,0);
        ch.otosDrive(50,-24,0);
        ch.otosDrive(50,-24,50);
        ch.otosDrive(50,-24,0);
        ch.otosDrive(5,-24,0);
        ch.otosDrive(0,0,0);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            ch.otosDrive(0,0,0);

        }
    }
}