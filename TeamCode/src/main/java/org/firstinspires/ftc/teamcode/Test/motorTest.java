package org.firstinspires.ftc.teamcode.Test;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="motorTest", group="Linear OpMode")
public class motorTest extends LinearOpMode {

    public DcMotor TestMotor1 = null;
    public DcMotor TestMotor2 = null;
    public ElapsedTime timer = new ElapsedTime();


    public void runOpMode() {

        TestMotor1 = hardwareMap.get(DcMotor.class, "testMotor1");

        TestMotor1.setDirection(DcMotor.Direction.REVERSE);
        TestMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TestMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        timer.reset();

        while (opModeIsActive()){

            TestMotor1.setPower(0.9);
            sleep(1000);
            TestMotor1.setPower(0);
            sleep(1500);
            TestMotor1.setPower(-0.9);
            sleep(1000);
            TestMotor1.setPower(0);




            telemetry.addData("Disticks", TestMotor1.getCurrentPosition());
            telemetry.update();

            break;
        }
        TestMotor1.setPower(0);
        sleep(10000);
    }

}