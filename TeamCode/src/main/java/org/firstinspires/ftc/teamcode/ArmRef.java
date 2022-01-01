package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ArmRef {
    public static double posA=150;
    public static double posB=0;
    public static double posX=0;
    public static double posY=630;
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
        public static double armVelocitySlope =100; //Based on position
        public static double armPowerSlope=1.0; //Based on velocity
        public static double armPowerLimit=0.7;
        public static double armVelocityLimit =0.3;
        public static double armInterp=0.8;
        public static double armVelInterp=0.2;
        public static double armPowAdjust =0.1;
    }
}
