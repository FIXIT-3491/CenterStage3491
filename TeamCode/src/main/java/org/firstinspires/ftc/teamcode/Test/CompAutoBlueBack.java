package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CaptainHook.Constants.CS;
import org.firstinspires.ftc.teamcode.CaptainHook.CH;
import org.firstinspires.ftc.teamcode.CaptainHook.VP;
@Disabled
@Autonomous(name="CompAutoBlueBack", group="Linear OpMode")

public class CompAutoBlueBack extends LinearOpMode {
    public CH ch = null;
    private VP vp = null;

    private ElapsedTime stepTimer = new ElapsedTime();
    private String Location;

    @Override
    public void runOpMode() {

        ch = new CH(hardwareMap, this);
        vp = new VP(hardwareMap, this);
        vp.initCompVision();

        ch.rightPincer.setPosition(CS.C_RIGHT_CLOSE);
        ch.wrist.setPosition(CS.WRIST_UP);

        telemetry.addData("Status", "initialized ");
        telemetry.update();

        ch.imu.resetYaw();
        waitForStart();

        stepTimer.reset();
        if (opModeIsActive())
        {
            TensorFlow();

            PurplePixel();

            YellowPixel();

            Park();

        } // if active
    } // run op mode
    public void TelemetryStep(String step) {
        telemetry.addData("Step", step);
        telemetry.addData("prop location", Location);
        telemetry.update();
    }
    public void TensorFlow(){
        ch.wrist.setPosition(CS.WRIST_DOWN);
        TelemetryStep("TensorDetect");
        Location = vp.TensorDetect();
        TelemetryStep("Move forward");
        ch.wrist.setPosition(0.13);
        ch.EncoderMove(750);
    }
    public void PurplePixel(){

        if (Location == "left") { // actrually right
            vp.DESIRED_TAG_ID = 1;
            TelemetryStep("Turn to left");
            ch.imuTurn(33);
            TelemetryStep("Move to left");
            ch.armMove(600);
            ch.EncoderMove(515);
            BackFromSpike(600);
            TelemetryStep("Turn to backdrop");
            ch.imuTurn(80);

        } else if (Location == "right") { //actually left
            vp.DESIRED_TAG_ID = 3;
            TelemetryStep("Turn to right");
            ch.imuTurn(-33);
            TelemetryStep("Move to right");
            ch.armMove(800);
            ch.EncoderMove(450);
            BackFromSpike(550);
            ch.armMove(0);
            TelemetryStep("Turn to backdrop");
            ch.imuTurn(80);

        } else {
            vp.DESIRED_TAG_ID = 2;
            TelemetryStep("Move to Center");

            ch.EncoderMove(CS.E_SPIKE_LEFT_CENTER);
            BackFromSpike(600);
            TelemetryStep("Turn to backdrop");
            ch.imuTurn(80);
        }
    }
    private void BackFromSpike(int amount){
        ch.wrist.setPosition(CS.WRIST_UP);
        TelemetryStep("Back from spike mark");
        ch.moveRobot(-0.5, 0, 0);
        sleep(amount);
        ch.moveRobot(0, 0, 0);
    }
    private void YellowPixel() {
        stepTimer.reset();
        ch.wrist.setPosition(0.15);
        vp.setManualExposure(6);
        ch.EncoderMove(200);
        TelemetryStep("Move april tag");
        ch.moveAprilTag(vp);
        ch.dropPixel1();
        sleep(1000);
        ch.spinnerIntake.setPower(0.3);
        ch.EncoderMove(400);
        sleep(500);
        ch.rightPincer.setPosition(CS.C_RIGHT_OPEN);
        sleep(500);
        ch.spinnerIntake.setPower(0);
    }
    public void Park() {

        ch.moveRobot(-0.5,0,0);
        sleep(500);
        ch.moveRobot(0,0,0);

        ch.closeArmAuto();
        ch.imuTurn(0);

        ch.moveRobot(-0.5,0,0);
        sleep(1500);
        ch.moveRobot(0,0,0);

        ch.moveRobot(0,0.5,0);
        sleep(1400);
        ch.moveRobot(0,0,0);
        ch.armExtender.setTargetPosition(0);
        ch.armExtender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ch.armExtender.setPower(1);


    }

} //linear op mode