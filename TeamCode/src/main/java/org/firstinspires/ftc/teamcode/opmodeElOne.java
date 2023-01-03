
package org.firstinspires.ftc.teamcode;

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
public class opmodeElOne extends LinearOpMode {
    //front right
    private DcMotor motorFR = null;
    //back right
    private DcMotor motorBR = null;
    //front left
    private DcMotor motorFL = null;
    //back left
    private DcMotor motorBL = null;
    //potentially reduce power, not written yet
    private double modulation = 1;

    @Override
    public void runOpMode() {
        //set up front right
        motorFR  = hardwareMap.get(DcMotor.class,  "testmotorFR");
        motorFR.setDirection(DcMotor.Direction.FORWARD);
        //set up back right
        motorBR = hardwareMap.get(DcMotor.class, "testmotorBR");
        motorBR.setDirection(DcMotor.Direction.FORWARD);
        //set up front left
        motorFL = hardwareMap.get(DcMotor.class, "testmotorFL");
        motorFL.setDirection(DcMotor.Direction.FORWARD);
        //set up back left
        motorBL = hardwareMap.get(DcMotor.class, "testmotorBL");
        motorBL.setDirection(DcMotor.Direction.FORWARD);

        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        while(opModeIsActive()){


            double leftPower;
            double rightPower;
            double drive = gamepad1.left_stick_y+ gamepad1.right_stick_y;
            double turnReduction = 1;
            double turn = (gamepad1.left_stick_x*turnReduction);  //Turning using the left stick.
            double strafe = (gamepad1.right_stick_x);  //Strafing using the right stick
            leftPower    = (drive - turn);
            rightPower   = (drive + turn);
            motorFL.setPower(Range.clip((leftPower+strafe)*modulation, -1.0, 1.0));
            motorFR.setPower(Range.clip((rightPower-strafe)*modulation, -1.0, 1.0));
            motorBL.setPower(Range.clip((leftPower-strafe)*modulation, -1.0, 1.0));
            motorBR.setPower(Range.clip((rightPower+strafe)*modulation, -1.0, 1.0));

                         /*
            //power of motors
            double motorPowerFR;
            double motorPowerBR;
            double motorPowerFL;
            double motorPowerBL;

            //bind all motor's power to left stick Y
            motorPowerFR = gamepad1.left_stick_y;
            motorPowerBR = gamepad1.left_stick_y;
            motorPowerFL = gamepad1.left_stick_y;
            motorPowerBL = gamepad1.left_stick_y;

            //set the power of each motor to the clipped power double
            motorFR.setPower(Range.clip(motorPowerFR,-1.0,1.0));
            motorBR.setPower(Range.clip(motorPowerBR,-1.0,1.0));
            motorFL.setPower(Range.clip(motorPowerFL,-1.0,1.0));
            motorBL.setPower(Range.clip(motorPowerBL,-1.0,1.0));

            //idk what this does
            telemetry.addData( "Motor Power Front Right", motorPowerFR);
            telemetry.addData( "Motor Power Back Right", motorPowerBR);
            telemetry.addData( "Motor Power Front Left", motorPowerFL);
            telemetry.addData( "Motor Power Back Left", motorPowerBL);
            */
        }

    }
}