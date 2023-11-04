package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class CH {

    public DcMotor backLDrive = null;
    public DcMotor frontLDrive = null;
    public DcMotor frontRDrive = null;
    public DcMotor backRDrive = null;
    public DcMotor winchMotor = null;
    public Servo hookArm = null;
    public Servo launcher = null;
    public static final double POS_1 = 0.35;
    public static final double POS_2 = 0.5;
    public static final double MIN_POS = 0.0;
    public static final double Fire = 0.5;
    public static final double Tight = -0.7;
    public static final double Loose = 0.7;

   //private Servo flyWheel1 = null;
   //private Servo flyWheel2 = null;



    public CH(HardwareMap hardwareMap){

     frontLDrive = hardwareMap.get(DcMotor.class, "frontL");
     backLDrive = hardwareMap.get(DcMotor.class, "backL");
     frontRDrive = hardwareMap.get(DcMotor.class, "frontR");
     backRDrive = hardwareMap.get(DcMotor.class, "backR");
     winchMotor = hardwareMap.get(DcMotor.class, "winch");
     hookArm = hardwareMap.get(Servo.class, "arm");
     launcher = hardwareMap.get(Servo.class, "launcher");

     frontLDrive.setDirection(DcMotor.Direction.REVERSE);
     backLDrive.setDirection(DcMotor.Direction.REVERSE);
     frontRDrive.setDirection(DcMotor.Direction.FORWARD);
     backRDrive.setDirection(DcMotor.Direction.FORWARD);

     frontLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
     frontRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
     backLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
     backRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
}

/*

    //flyWheel1 = hardwareMap.get(Servo.class,"flyWheel1");
    //flyWheel2 = hardwareMap.get(Servo.class, "flyWheel2");


    //flyWheel1.setDirection(Servo.Direction.REVERSE);

    hookArm.setPosition(MIN_POS);
    launcher.setPosition(0);
*/