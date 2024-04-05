package org.firstinspires.ftc.teamcode.Test;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.setup.CH;

@Autonomous(name="motorTest", group="Linear OpMode")
public class motorTest extends LinearOpMode {

    public CH ch = null;
    public ElapsedTime timer = new ElapsedTime();


    public void runOpMode() {
        ch = new CH(hardwareMap, this);
        waitForStart();
        timer.reset();

        while (opModeIsActive()){

            ch.EncoderMove2(200,0);
            break;
        }
        sleep(10000);
    }

}