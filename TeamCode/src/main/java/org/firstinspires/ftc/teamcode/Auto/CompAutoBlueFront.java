package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CaptainHook.Constants.CS;
import org.firstinspires.ftc.teamcode.CaptainHook.CH;
import org.firstinspires.ftc.teamcode.CaptainHook.VP;

@Autonomous(name="CompAutoBlueFront", group="Linear OpMode")

public class CompAutoBlueFront extends LinearOpMode {
    public CH ch = null;
    private VP vp = null;
    private ElapsedTime stepTimer = new ElapsedTime();
    private String Location;

    @Override
    public void runOpMode() {

        ch.imu.resetYaw();

        ch = new CH(hardwareMap, this);
        vp = new VP(hardwareMap, this);

        vp.initCompVision();

        telemetry.addData("Status", "initialized ");
        telemetry.update();

        waitForStart();

        stepTimer.reset();
        if (opModeIsActive())
        {
            Location = vp.TensorDetect();


            ch.EncoderMove(750);

            if (Location == "left") {
                ch.imuTurn(57);
                ch.EncoderMove(CS.E_SPIKE_RIGHT);
            } else if (Location == "right") {
                ch.imuTurn(-50);
                ch.EncoderMove(CS.E_SPIKE_RIGHT);
            } else {
                ch.EncoderMove(CS.E_SPIKE_LEFT_CENTER);
            }
            ch.moveRobot(-0.4, 0, 0);
            sleep(500);
            ch.moveRobot(0, 0, 0);


        } // if active
    } // run op mode
} //linear op mode