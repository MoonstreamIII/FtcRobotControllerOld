package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ArmRef {
    public static double posA=150;
    public static double posB=0;
    public static double posX=790;
    public static double posY=590;
    public static double adjust=0.0001;
    public static boolean autoArm=true;
    @Config
    public static class PowerMode {
        public static double armError=0;
        public static double armDistMultiplier=100.0;
        public static double tgtArmPower=0.1;
        public static double armPowerLimit=0.3;
    }
    @Config
    public static class EncoderMode {
        public static double armPowerLimit=0.5;
        public static double armPowerSlope=50;
    }
    @Config
    public static class VelocityMode {
        public static double armVelocitySlope =150; //Based on position
        public static double armPowerSlope=1.5 ; //Based on velocity
        public static double armPowerLimit=0.35;
        public static double armVelocityLimit =0.5;
        public static double armInterp=1;
        public static double armVelInterp=1;
        public static double armPowAdjust =0.05;
    }
    @Config
    public static class GravMode {
        public static int zeroPos=460;
        public static double gravPow=0.4;
        public static double sidewaysDist =320;
    }
}
