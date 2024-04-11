package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.CaptainHook.CH;
import org.firstinspires.ftc.teamcode.CaptainHook.Constants;

@TeleOp(name="Setup", group="Linear OpMode")

public class Setup extends LinearOpMode {

    private CH ch = null;


    public boolean dropPixelCalled = false;

    @Override
    public void runOpMode() {

        ch = new CH(hardwareMap, this);
        ch.backLDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ch.backRDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        boolean launchToggle = true;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            double max;

            if (gamepad2.dpad_up)  // tighten
                ch.winchMotor.setPower(Constants.CS.WINCH_TIGHTEN);
            else if (gamepad2.dpad_down)  // loosen winch
                ch.winchMotor.setPower(Constants.CS.WINCH_LOOSEN);
            else if (gamepad2.dpad_left || gamepad2.dpad_right)  // hold winch
                ch.winchMotor.setPower(0.2);
            else  //zero out winch
                ch.winchMotor.setPower(0);

            if (gamepad2.a) {
                ch.launcher.setPosition(0.7);

            } if (gamepad2.b){
                ch.launcher.setPosition(0.19);

            }
        }
    }
}

