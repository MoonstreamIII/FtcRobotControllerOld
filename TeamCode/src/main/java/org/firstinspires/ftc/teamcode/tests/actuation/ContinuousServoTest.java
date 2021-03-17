package org.firstinspires.ftc.teamcode.tests.actuation;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.core.gamepad.GamepadEventPS;

@TeleOp(name = "CR Servo Test", group = "Tests")
public class ContinuousServoTest extends OpMode {
    CRServo servo;
    double power = 0;
    GamepadEventPS update;

    @Override
    public void init() {
        servo = hardwareMap.crservo.get("wobbleArm");
//        servo.getController().setServoPosition(0,1);
        update = new GamepadEventPS(this.gamepad1);

    }

    @Override
    public void loop() {
//        if
        if(gamepad1.circle)
            power = -.2;
        if(gamepad1.square)
            power = -.33;
        if(gamepad1.triangle)
            power = -.46;

        if (update.dPadLeft())
            power -= .01;
        if (update.dPadRight())
            power += .01;
        servo.setPower(power);
        telemetry.addData("power", power);
    }
}
