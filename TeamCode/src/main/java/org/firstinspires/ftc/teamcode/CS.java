package org.firstinspires.ftc.teamcode;
//first and last letter naming scheme
public class CS { //Constants
    public static class RT { // Robot stuff
        public static final double A_SPEED_GAIN =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
        public static final double A_STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
        public static final double A_TURN_GAIN =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

        public static final double MAX_AUTO_SPEED = 0.2;   //  Clip the approach speed to this max value (adjust for your robot)
        public static final double MAX_AUTO_STRAFE= 0.2;   //  Clip the approach speed to this max value (adjust for your robot)
        public static final double MAX_AUTO_TURN  = 0.2;   //  Clip the turn speed to this max value (adjust for your robot)

        public static final double E_INCREMENT = 0.01;     // amount to ramp motor each CYCLE_MS cycle
        public static final int    E_CYCLE_MS = 25;     // period of each cycle
        public static final double E_MAX_POWER = 0.4;     // Maximum FWD power applied to motor
        public static final double E_MIN_POWER = 0.1;     // Maximum REV power applied to motor
        public static final int E_SPIKE_LEFT_RIGHT = 500;
        public static final int E_SPIKE_LEFT_CENTER = 800;

        //public static final double armMIN_POS = 0.25; // old arm poz
        //public static final double armPOS_1 = 0.52;
        //public static final double armPOS_2 = 0.65;

        public static final double W_TIGHTEN = -0.7;
        public static final double W_LOOSEN = 0.7;
    }

    public static class CV { // Computer Vision stuff
        public final double DESIRED_DISTANCE = 11.0; //  this is how close the camera should get to the target (inches)
    }
}
