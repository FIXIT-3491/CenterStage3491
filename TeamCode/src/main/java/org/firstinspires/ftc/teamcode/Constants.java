package org.firstinspires.ftc.teamcode;
//first and last letter naming scheme
public class Constants { //Constants
    public static class CS { // Robot stuff
        public static final double A_SPEED_GAIN =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
        public static final double A_STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
        public static final double A_TURN_GAIN =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
        public static final double A_DESIRED_DISTANCE = 16.0; //  this is how close the camera should get to the target (inches)

        public static final double A_MAX_AUTO_SPEED = 0.2;   //  Clip the approach speed to this max value (adjust for your robot)
        public static final double A_MAX_AUTO_STRAFE = 0.2;   //  Clip the approach speed to this max value (adjust for your robot)
        public static final double A_MAX_AUTO_TURN = 0.4;   //  Clip the turn speed to this max value (adjust for your robot)

        public static final double E_INCREMENT = 0.01;     // amount to ramp motor each CYCLE_MS cycle
        public static final int    E_CYCLE_MS = 25;     // period of each cycle
        public static final double E_MAX_POWER = 0.5;     // Maximum FWD power applied to motor
        public static final double E_MIN_POWER = 0.1;     // Maximum REV power applied to motor
        public static final int    E_SPIKE_LEFT_RIGHT = 500;
        public static final int    E_SPIKE_LEFT_CENTER = 675;

        public static final double WINCH_TIGHTEN = 0.7; //winch
        public static final double WINCH_LOOSEN = -0.7;

        public static final double C_RIGHT_CLOSE = 0.45;
        public static final double C_LEFT_CLOSE = 0.5;
        public static final double C_RIGHT_OPEN = 0.85;
        public static final double C_LEFT_OPEN = 0.2;

        public static final int ARM_UP = 1300;
        public static final int ARM_DOWN = 50;
        public static final int ARM_DOWN_EXT = 500;
        public static final int ARM_MAX = 2100;

        public static final int EXT_MAX = 1300;
        public static final int EXT_POS2 = 1200;
        public static final int EXT_POS1 = 700;
        public static final int EXT_MIN = 0;



        public static final double WRIST_DOWN = 0.1;
        public static final double WRIST_UP = 0.45;
        public static final double WRIST_LINE_1 = 0.5;
        public static final double WRIST_LINE_2 = 0.56;
    }
}
