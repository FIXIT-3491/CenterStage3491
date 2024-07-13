package org.firstinspires.ftc.teamcode.Auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CaptainHook.CH;
import org.firstinspires.ftc.teamcode.CaptainHook.Constants.CS;
import org.firstinspires.ftc.teamcode.CaptainHook.VP;

@Autonomous
public class BackRed extends LinearOpMode {
    private CH ch = null;
    private VP vp = null;
    private ElapsedTime time = new ElapsedTime();

    private String Location;

    @Override
    public void runOpMode() {
        ch = new CH(hardwareMap, this);
        vp = new VP(hardwareMap, this);

        vp.initCompVision();
        ch.configureOtos();

        ch.backLDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ch.backRDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ch.wrist.setPosition(CS.WRIST_UP);
        sleep(1000);

        ch.leftPincer.setPosition(CS.C_LEFT_CLOSE);
        ch.rightPincer.setPosition(CS.C_RIGHT_CLOSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();



        waitForStart();

        Location = vp.TensorDetect();



        ch.wrist.setPosition(CS.WRIST_DOWN);
        ch.wrist.setPosition(CS.WRIST_UP);

        ch.scorePurplePixel(Location, "backRed");

        ch.leftPincer.setPosition(CS.C_LEFT_OPEN);
//            sleep(500);
        ch.wrist.setPosition(CS.WRIST_UP);

        sleep(4000);



    }
}
