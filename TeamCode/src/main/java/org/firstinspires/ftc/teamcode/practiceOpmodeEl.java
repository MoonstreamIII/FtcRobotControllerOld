
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/*
 * Test the dashboard gamepad integration.
 */
//@Disabled
@TeleOp(name="El's OpMode", group="Skybot")
public class practiceOpmodeEl extends LinearOpMode {
    private DcMotor motorR = null;
    private DcMotor motorL = null;

    @Override
    public void runOpMode() {
        motorR  = hardwareMap.get(DcMotor.class,  "testmotorR");
        motorR.setDirection(DcMotor.Direction.FORWARD);
        motorL = hardwareMap.get(DcMotor.class, "testmotorL");
        motorL.setDirection(DcMotor.Direction.FORWARD);
        motorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        while(opModeIsActive()){
            double motorPowerL;
            double motorPowerR;
            motorPowerL = gamepad1.left_stick_y;
            motorPowerR = gamepad1.right_stick_y;
            motorR.setPower(Range.clip(motorPowerR,-1.0,1.0));
            motorL.setPower(Range.clip(motorPowerL,-1.0,1.0));
            telemetry.addData( "Motor Power Left", motorPowerL);
            telemetry.addData( "Motor Power Right", motorPowerR);
        }

    }
}