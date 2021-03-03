package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.core.StandardMechanumDrive;
import org.firstinspires.ftc.teamcode.core.gamepad.GamepadEventPS;

import static java.lang.Math.PI;
import static java.lang.Math.cos;
import static java.lang.Math.pow;
import static java.lang.Math.sqrt;
import static java.lang.Math.tan;
import static org.firstinspires.ftc.teamcode.TeleOp.powerScale;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.FEEDER_REST;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.FEEDER_YEET;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.FLYWHEEL_RADIUS;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.LAUNCHER_ANGLE;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.TOWER_GOAL_VERTICAL_DISPLACEMENT;

@Config
@TeleOp(name = "Kobe Test Part 2")
public class KobeTest2 extends OpMode {
    DcMotorEx shooter;
    Servo feeder;
    final double wheelRadius = 0.0508; // in meters
    double angularVelocity = 0; // rad/s
    double linearVelocity = 0; // m/s

    final double MAX_RPM = 6000.0; // in RPM, if you couldn't figure it out
    final double MAX_RAD = MAX_RPM * ((2.0 * PI) / 60.0);
    final double MAX_LINEAR_SPEED = MAX_RAD * wheelRadius; // m/s

    double distance = 73;

    StandardMechanumDrive drive;
    public static PIDFCoefficients shooterPIDF = new PIDFCoefficients(15,3,19,12);

    GamepadEventPS update;
    @Override
    public void init() {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        feeder = hardwareMap.servo.get("feeder");
        feeder.setPosition(FEEDER_REST);
        update = new GamepadEventPS(gamepad1);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setVelocityPIDFCoefficients(.2,1,0,5);
        drive = new StandardMechanumDrive(this.hardwareMap);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);
    }

    @Override
    public void loop() {

        if(update.dPadUp())
            angularVelocity += .1;
        if(update.dPadDown())
            angularVelocity -= .1;

        if(update.dPadLeft())
            distance -= 2;
        if(update.dPadRight())
            distance += 2;

        if(gamepad1.left_bumper)
            feeder.setPosition(FEEDER_YEET);
        else
            feeder.setPosition(FEEDER_REST);

        drive.setDrivePower(
                new Pose2d(
                        powerScale(gamepad1.left_stick_y, .35),
                        powerScale(gamepad1.left_stick_x, .35),
                        -powerScale(gamepad1.right_stick_x, .35)
                )
        );

        linearVelocity = shooter.getVelocity(AngleUnit.RADIANS) * wheelRadius;


        shooter.setVelocity(angularVelocity, AngleUnit.RADIANS);
        telemetry.addLine("Use dPad Up/Down to change motor speed");
        telemetry.addData("Angular velocity (rad)", angularVelocity);
//        telemetry.addData("Angular Velocity (deg)", toDegrees(angularVelocity));
//        telemetry.addData("Linear Velocity", linearVelocity);
        telemetry.addData("Motor Power", shooter.getPower());
        telemetry.addData("Actual velocity (rad/s)", shooter.getVelocity(AngleUnit.RADIANS));
//        telemetry.addData("Distance from goal", distance);
//        telemetry.addData("calculated speed (set it to this)", calcInitialSpeed());
        telemetry.update();
    }
    double calcInitialSpeed() {
        double h = TOWER_GOAL_VERTICAL_DISPLACEMENT; // in meters
        final double g = 9.8; // In m/s^2
        double d = distance; // in meters
        double linearSpeed = sqrt((pow(d, 2) * g) / (pow(cos(LAUNCHER_ANGLE), 2) * (2 * d * tan(LAUNCHER_ANGLE) - (2 * h) )));
        return linearSpeed;// / FLYWHEEL_RADIUS;
    }
}
