package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ArmRef {
    public static double posA=150;
    public static double posB=0;
    public static double posX=0;
    public static double posY=630;
    public static class PowerMode {
        public static double armError=0;
        public static double armDistMultiplier=100.0;
        public static double tgtArmPower=0.1;
        public static double armPowerLimit=0.3;
    }
    public static class SpeedMode {
        public static double speedDistMultiplier=0;
        public static double powerChangeMultiplier=0;
        public static double armPowerLimit=0.3;
        public static double baseArmPower=0;
        public static double baseArmSpeed=0;
    }
}
