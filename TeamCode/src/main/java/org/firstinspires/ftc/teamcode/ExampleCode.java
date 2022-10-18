package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//This is a system where the triggers control the speed of the bot. For example, if you have the left stick at full forward and the left trigger half pressed in, you get half power.
@SuppressWarnings("FieldCanBeLocal")
@TeleOp(name="ExampleCode",group="Linear")
//@Disabled
public class ExampleCode extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    //lfd is left front drive
    //rfd is right front drive
    //lbd is left back drive
    //rbd is right back drive
    private DcMotor lfd = null;
    private DcMotor rfd = null;
    private DcMotor lbd = null;
    private DcMotor rbd = null;
    private double modulation = 0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Load the motors into the motor variables. (The names are in HardwareReference)
        lfd  = hardwareMap.get(DcMotor.class, HardwareReference.LEFT_FRONT_DRIVE);
        rfd = hardwareMap.get(DcMotor.class, HardwareReference.RIGHT_FRONT_DRIVE);
        lbd  = hardwareMap.get(DcMotor.class, HardwareReference.LEFT_REAR_DRIVE);
        rbd = hardwareMap.get(DcMotor.class, HardwareReference.RIGHT_REAR_DRIVE);
        //Setting motor directions and telling the motors to use their encoders to control their speed.
        lfd.setDirection(DcMotor.Direction.FORWARD);
        rfd.setDirection(DcMotor.Direction.REVERSE);
        lbd.setDirection(DcMotor.Direction.FORWARD);
        rbd.setDirection(DcMotor.Direction.REVERSE);
        lfd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rfd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lbd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rbd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //Wait until "play" button is pressed.
        waitForStart();
        runtime.reset();
        //Run until stop is pressed
        while (opModeIsActive()) {
            //Get motor target values
            double leftPower;
            double rightPower;
            double drive = gamepad1.left_stick_y+ gamepad1.right_stick_y;
            double turnReduction = 1;
            double turn = (gamepad1.left_stick_x*turnReduction);  //Turning using the left stick.
            double strafe = (gamepad1.right_stick_x);  //Strafing using the right stick
            leftPower    = (drive - turn);
            rightPower   = (drive + turn);
            lfd.setPower(Range.clip((leftPower+strafe)*modulation, -1.0, 1.0));
            rfd.setPower(Range.clip((rightPower-strafe)*modulation, -1.0, 1.0));
            lbd.setPower(Range.clip((leftPower-strafe)*modulation, -1.0, 1.0));
            rbd.setPower(Range.clip((rightPower+strafe)*modulation, -1.0, 1.0));



        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        telemetry.update();
        }
    }
}