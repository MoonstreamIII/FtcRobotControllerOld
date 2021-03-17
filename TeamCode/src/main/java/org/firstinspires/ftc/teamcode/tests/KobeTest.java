package org.firstinspires.ftc.teamcode.tests;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.core.gamepad.GamepadEventPS;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

@Disabled
@TeleOp
public class KobeTest extends OpMode {
    DcMotor shooter;
    GamepadEventPS update;
    double power = 0;
    Servo feeder;
    double feederPosition = 0;
    final double initial = .34;
    final double launch = .38;
    VoltageSensor voltage;
    double currentPower;
    List<Double> slopes = new LinkedList<Double>();
    Map<Double, Double> posVals = new HashMap<>();

    @Override
    public void init() {
        update = new GamepadEventPS(gamepad1);
        shooter = this.hardwareMap.get(DcMotor.class, "shooter");
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        feeder = hardwareMap.servo.get("feeder");
        voltage = hardwareMap.voltageSensor.iterator().next();
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void loop() {
        double Φ = 9.9;

        if(update.rightBumper())
            feeder.setPosition(launch);

        if(update.leftBumper())
            feeder.setPosition(initial);

        if(gamepad1.square)
            power = 1;

        if(gamepad1.circle)
            power = 0;

        if(update.dPadLeft()) {
            if(Math.abs(power) >= 0.01)
                power -= .01;
        }

        if(update.dPadRight()) {
            if(Math.abs(power) <= 1)
                power += .01;
        }

        if(update.dPadUp())
            feederPosition += .01;

        if(update.dPadDown())
            feederPosition -= .01;

        // Obselete
        /*if(shooter.getPower() != 0) {

            if(shooter.getPower() == currentPower) {
                posVals.put(time, (double) shooter.getCurrentPosition());

                if(posVals.size() >= 100) {
                    Map<Double, Double> sorted = MapsKt.toSortedMap(posVals);
                    double[] keys = CollectionsKt.toDoubleArray(sorted.keySet());
                    double[] values = CollectionsKt.toDoubleArray(sorted.values());
                    for(int i = 1; i < sorted.size() - 1; i++) {
                        slopes.add(slope(keys[i], keys[i - 1], values[i], values[i - 1]));
                    }
                    telemetry.addData("Standard deviation of slopes", sd(slopes));
                    telemetry.addData("Slope average", CollectionsKt.averageOfDouble(slopes));
                    telemetry.update();
                }
            }
            else {
                posVals.clear();
            }
        }*/

        currentPower = shooter.getPower();
        shooter.setPower(power);
        feeder.setPosition(feederPosition);
        telemetry.addData("Shooter power", power);
        telemetry.addData("square to put in feed pos", "triangle to put in initial pos");
        telemetry.addData("feeder pos", feeder.getPosition());
        telemetry.addData("voltage", voltage.getVoltage());
        telemetry.update();

    }
    double slope(double x2, double x1, double y2, double y1) {
        return (y2 - y1) / (x2 - x1);
    }
    double sd(List<Double> numArray) {
        double sum = 0.0;
        double standardDeviation = 0.0;

        for (double num : numArray) {
            sum += num;
        }

        double mean = sum / 10;

        for (double num : numArray) {
            standardDeviation += Math.pow(num - mean, 2.0);
        }
        return Math.sqrt(standardDeviation / 10);
    }

}
