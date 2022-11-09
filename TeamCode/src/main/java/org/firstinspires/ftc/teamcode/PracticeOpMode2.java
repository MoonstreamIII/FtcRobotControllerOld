
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;






@TeleOp(name="PracticeOpMode 2", group="Skybot")
@Disabled
public class PracticeOpMode2 extends LinearOpMode {
    private DcMotor motor=null; //Arm motor
    @Override
    public void runOpMode() {
        motor=hardwareMap.get(DcMotor.class, "testmotor");
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();
        while (opModeIsActive()) {
            double motorPower;
            motorPower=gamepad1.left_stick_y;
            if (gamepad1.x) {
                motorPower=1;
            }
            motor.setPower(Range.clip(motorPower,-1.0,1.0));
            telemetry.addData("Motor Power",motorPower);
        }
    }
}