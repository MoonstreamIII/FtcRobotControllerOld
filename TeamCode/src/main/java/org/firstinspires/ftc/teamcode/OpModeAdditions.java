
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
    private double grabPos = 0.25;
    private double pos = 0;
    //potentially reduce power, not written yet
    private double modulation = 1;
    private double modLow = 0.5;
    private double modHigh = 1.0;
    private boolean grabtoggle=false;
    private boolean grabOn=false;
    private double maxLiftSpeed=0.5;
    private boolean velMode=false;
    private int posX = 0;
    private int posY = 0;
    private int posA = 0;
    private int posB = 0;
    private int liftJoyMul=10;
    private int liftSlowRange=10;
    private double liftSlowSpeed=0.1;
    private int liftPos=0;

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
        liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftL.setTargetPosition(0);
        liftR.setTargetPosition(0);
        liftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();
        while(opModeIsActive()){
            double liftSpeedL;
            double liftSpeedR;
            if (gamepad1.left_bumper) { modulation=modLow; }
            if (gamepad1.right_bumper) { modulation=modHigh; }
            if (gamepad2.left_bumper) {grabOn=false;}
            if (gamepad2.right_bumper) {grabOn=true;}
            if (gamepad2.x) { liftPos=posX; }
            if (gamepad2.y) { liftPos=posY; }
            if (gamepad2.a) { liftPos=posA; }
            if (gamepad2.b) { liftPos=posB; }
            if (gamepad1.left_trigger>0||gamepad1.right_trigger>0) {
                if (!velMode) {
                    velMode = true;
                    liftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    liftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
            } else if (velMode) {
                liftPos=liftL.getCurrentPosition();
                velMode = false;
                liftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (!velMode) {
                liftL.setTargetPosition(liftPos);
                liftR.setTargetPosition(liftPos);
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
            if (velMode) {
                liftSpeedL=gamepad1.right_trigger-gamepad1.left_trigger;
                liftSpeedR=gamepad1.right_trigger-gamepad1.left_trigger;
            } else {
                if (Math.abs(liftPos - liftL.getCurrentPosition()) < liftSlowRange) {
                    liftSpeedL = liftSlowSpeed + (maxLiftSpeed - liftSlowSpeed) * (liftPos - liftL.getCurrentPosition());
                } else {
                    liftSpeedL = maxLiftSpeed;
                }
                if (Math.abs(liftPos - liftR.getCurrentPosition()) < liftSlowRange) {
                    liftSpeedR = liftSlowSpeed + (maxLiftSpeed - liftSlowSpeed) * (liftPos - liftR.getCurrentPosition());
                } else {
                    liftSpeedR = maxLiftSpeed;
                }
            }
            motorFL.setPower(Range.clip((leftPower+strafe)*modulation, -1.0, 1.0));
            motorFR.setPower(Range.clip((rightPower-strafe)*modulation, -1.0, 1.0));
            motorBL.setPower(Range.clip((leftPower-strafe)*modulation, -1.0, 1.0));
            motorBR.setPower(Range.clip((rightPower+strafe)*modulation, -1.0, 1.0));
            liftL.setPower(liftSpeedL);
            liftR.setPower(liftSpeedR);
            telemetry.addData("LiftPos", liftPos);
            telemetry.update();
        }

    }
}