package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class CustomAuto {
    @Config
    public static class testAuto {
        public static String[] commands={"left","forward","posArm"};
        public static double[] values={1.0,0.5,520};
        public static double[] times={1000,0,1000};
        public static int[] encodes={0,100,0};
    }
}
