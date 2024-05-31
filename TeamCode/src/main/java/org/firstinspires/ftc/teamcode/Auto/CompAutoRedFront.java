package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CaptainHook.Constants;
import org.firstinspires.ftc.teamcode.CaptainHook.Constants.CS;
import org.firstinspires.ftc.teamcode.CaptainHook.CH;
import org.firstinspires.ftc.teamcode.CaptainHook.VP;

@Autonomous(name="CompAutoRedFront", group="Linear OpMode")

public class CompAutoRedFront extends LinearOpMode {
    public CH ch = null;
    private VP vp = null;

    private ElapsedTime stepTimer = new ElapsedTime();
    private String Location;



    @Override
    public void runOpMode() {

        ch = new CH(hardwareMap, this);
        vp = new VP(hardwareMap, this);
        vp.initCompVision();

        ch.armEncoderReset();
        ch.rightPincer.setPosition(0.4);
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

            vp.DESIRED_TAG_ID = 8;

            if (Location != "left") {
                ch.WhitePixel();

                vp.setManualExposure(6);

                ch.moveAprilTag2(vp);

                PickUpWhitePixel();
                DriveThroughTruss();
            }
            else {


                ch.imuTurn(-80);
                ch.moveRobot(0.7,0,0);
                sleep(1800);
                ch.moveRobot(0,0,0);
                ch.imuTurn(-115);
                ch.closeArmAuto();
            };;

            SetAprilTag();

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
        if (Location == "left") {
            TelemetryStep("Turn to left");
            ch.imuTurn(44);
            TelemetryStep("Move to left");
            ch.EncoderMove(350);
            BackFromSpike(450);
            TelemetryStep("Turn to backdrop");
            ch.imuTurn(7);
            ch.EncoderMove(1200);

        } else if (Location == "right") {
            TelemetryStep("Turn to right");
            ch.imuTurn(-44);
            TelemetryStep("Move to right");
            ch.EncoderMove(215);
            BackFromSpike(700);
            TelemetryStep("Turn to backdrop");
            ch.imuTurn(70);

        } else {
            ch.moveRobot(0,0.5,0);
            sleep(500);
            ch.moveRobot(0,0,0);
            TelemetryStep("Move to Center");
            ch.EncoderMove(480);
            ch.moveRobot(0,-0.5,0);
            sleep(500);
            ch.moveRobot(0,0,0);
            BackFromSpike(600);
            TelemetryStep("Turn to backdrop");
            ch.imuTurn(70);

        }
    }
    private void BackFromSpike(int amount){
        ch.wrist.setPosition(Constants.CS.WRIST_UP);
        TelemetryStep("Back from spike mark");
        ch.moveRobot(-0.5, 0, 0);
        sleep(amount);
        ch.moveRobot(0, 0, 0);
    }
    public void PickUpWhitePixel(){
        ch.moveRobot(0.2,0,0);
        sleep(150);
        ch.moveRobot(0,0,0);

        ch.leftPincer.setPosition(CS.C_LEFT_CLOSE);
        sleep(1000);
        ch.moveRobot(-0.5,0,0);
        sleep(200);
        ch.moveRobot(0,0,0);
        ch.wrist.setPosition(CS.WRIST_UP);
        ch.imuTurn(0);
    }
    public void DriveThroughTruss(){
        ch.EncoderMove(1000);
        ch.imuTurn(-85);
        ch.moveRobot(0.7,0,0);
        sleep(2000);
        ch.moveRobot(0,0,0);
        ch.imuTurn(-115);
        ch.closeArmAuto();
    }
    private void SetAprilTag(){
        if (Location == "left") {
            vp.DESIRED_TAG_ID = 4;
        }
        else if(Location == "right"){
            vp.DESIRED_TAG_ID = 6;
        }
        else {
            vp.DESIRED_TAG_ID = 5;
        }

    }

    private void YellowPixel(){
        stepTimer.reset();
        ch.wrist.setPosition(0.15);

        TelemetryStep("Move april tag");
        ch.EncoderMove(200);
        ch.moveAprilTag(vp);
        ch.shoulder.setTargetPosition(500);
        ch.shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ch.shoulder.setPower(0.8);
        sleep(1000);
        ch.EncoderMove(400);
        sleep(500);
        ch.rightPincer.setPosition(Constants.CS.C_RIGHT_OPEN);
        ch.leftPincer.setPosition(CS.C_LEFT_OPEN);
    }
    public void Park() {

        ch.moveRobot(-0.5,0,0);
        sleep(200);
        ch.moveRobot(0,0,0);

        ch.closeArmAuto();
        ch.imuTurn(0);

        ch.armExtender.setTargetPosition(0);
        ch.armExtender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ch.armExtender.setPower(1);

    }
} //linear op mode