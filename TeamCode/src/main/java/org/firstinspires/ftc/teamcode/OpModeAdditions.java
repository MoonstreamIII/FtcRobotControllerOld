
package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/*
 * Test the dashboard gamepad integration.
 */
//@Disabled

@TeleOp(name="OpMode(I hope)", group="Skybot")
public class OpModeAdditions extends LinearOpMode {
    //front right
    private DcMotor motorFR = null;
    //back right
    private DcMotor motorBR = null;
    //front left
    private DcMotor motorFL = null;
    //back left
    private DcMotor motorBL = null;
    private Servo grabber = null;
    private DcMotor liftL = null;
    private DcMotor liftR = null;
    private double relPos = 0;
    private double grabPos = 1.0;
    private double pos = 0;
    //potentially reduce power, not written yet
    private double modulation = 1;
    private double modLow = 0.5;
    private double modHigh = 1.0;
    private boolean grabtoggle=false;
    private boolean grabOn=false;

    @Override
    public void runOpMode() {
        //set up front right
        motorFR  = hardwareMap.get(DcMotor.class,  "rfd");
        motorFR.setDirection(DcMotor.Direction.FORWARD);
        //set up back right
        motorBR = hardwareMap.get(DcMotor.class, "rbd");
        motorBR.setDirection(DcMotor.Direction.FORWARD);
        //set up front left
        motorFL = hardwareMap.get(DcMotor.class, "lfd");
        motorFL.setDirection(DcMotor.Direction.FORWARD);
        //set up back left
        motorBL = hardwareMap.get(DcMotor.class, "lbd");
        motorBL.setDirection(DcMotor.Direction.FORWARD);

        liftL = hardwareMap.get(DcMotor.class, "liftL");
        liftL.setDirection(DcMotor.Direction.FORWARD);
        liftR = hardwareMap.get(DcMotor.class, "liftR");
        liftR.setDirection(DcMotor.Direction.FORWARD);
        grabber = hardwareMap.get(Servo.class, "grabber");

        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        while(opModeIsActive()){
            if (gamepad1.left_bumper) { modulation=modLow; }
            if (gamepad1.right_bumper) { modulation=modHigh; }
            if (gamepad2.x&&!grabtoggle) {
                grabOn=!grabOn;
                grabtoggle=true;
            } else if (!gamepad2.x) {
                grabtoggle=false;
            }
            grabber.setPosition(grabOn? grabPos : relPos);
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
            liftL.setPower(gamepad2.left_stick_y);
            liftR.setPower(gamepad2.left_stick_y);

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