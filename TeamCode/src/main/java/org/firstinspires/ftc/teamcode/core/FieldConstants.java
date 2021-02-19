package org.firstinspires.ftc.teamcode.core;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import static java.lang.Math.toRadians;

@Config
public class FieldConstants {
    public static double MAX_EDGE = 72;
    public static double SHOOT_LINE = 0; // Theoretical: 12

    // For shooting coordinates -30 -25 10
    public static Vector2d redGoal = new Vector2d(MAX_EDGE, -44);
    public static Vector2d leftPowerShot = new Vector2d(MAX_EDGE, -4.5);
    public static Vector2d centerPowerShot = new Vector2d(MAX_EDGE, -9);
    public static Vector2d rightPowerShot = new Vector2d(MAX_EDGE, -13.5);

    // For path coordinates
    public static Vector2d ringPos = new Vector2d(-24, -36);
    public static Pose2d centerA = new Pose2d(42,-56, toRadians(90));
    public static Pose2d centerB = new Pose2d(46,-23, toRadians(90));
    public static Pose2d centerC = new Pose2d(70, -65, toRadians(90)); // theoretical: x = 60, y = -60

    public static Pose2d backPoseA = new Pose2d(-49.25,-32, toRadians(0));
    public static Pose2d backPoseB = new Pose2d(-47.5, -44, toRadians(0)); //theoretical: x = -31, y = -48
    public static Pose2d backPoseC = new Pose2d(-52,-26, toRadians(0));

    public static Pose2d autonStartPose = new Pose2d(-63, -33, toRadians(0));
}
