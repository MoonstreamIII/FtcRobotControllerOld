package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@SuppressWarnings("FieldCanBeLocal")
@TeleOp(name="Demo Op Mode - Linear",group="Linear")
@Disabled
public class TableBot_LinearArmExample extends LinearOpMode {
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
    private DcMotor armTop = null;
    private DcMotor armBottom = null;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        lfd  = hardwareMap.get(DcMotor.class, HardwareReference.LEFT_FRONT_DRIVE);
        rfd = hardwareMap.get(DcMotor.class, HardwareReference.RIGHT_FRONT_DRIVE);
        lbd  = hardwareMap.get(DcMotor.class, HardwareReference.LEFT_REAR_DRIVE);
        rbd = hardwareMap.get(DcMotor.class, HardwareReference.RIGHT_REAR_DRIVE);
        armBottom = hardwareMap.get(DcMotor.class, HardwareReference.ARM_BOTTOM);
        armTop = hardwareMap.get(DcMotor.class, HardwareReference.ARM_TOP);
        //foo = hardwareMap.get(DcMotor.class, "foo_motor");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        lfd.setDirection(DcMotor.Direction.FORWARD);
        rfd.setDirection(DcMotor.Direction.REVERSE);
        lbd.setDirection(DcMotor.Direction.FORWARD);
        rbd.setDirection(DcMotor.Direction.REVERSE);
        armBottom.setDirection(DcMotor.Direction.FORWARD);
        armTop.setDirection(DcMotor.Direction.FORWARD);

        armTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armBottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;
            double armTopPower;
            double armBottomPower;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            //double drive = Math.max( gamepad1.left_stick_y, Math.max(gamepad1.right_stick_y, gamepad1.right_trigger - gamepad1.left_trigger));
            double turn = -( gamepad1.left_stick_x);  //Turning using the left stick.
            double strafe = (gamepad1.right_stick_x);  //Strafing using the right stick.
            leftPower    = drive - turn;
            rightPower   = drive + turn;
            armBottomPower = gamepad2.left_stick_y;
            armTopPower = gamepad2.right_stick_y;


            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels
            lfd.setPower(Range.clip(leftPower+strafe, -1.0, 1.0));
            rfd.setPower(Range.clip(rightPower-strafe, -1.0, 1.0));
            lbd.setPower(Range.clip(leftPower-strafe, -1.0, 1.0));
            rbd.setPower(Range.clip(rightPower+strafe, -1.0, 1.0));
            armTop.setPower(armTopPower);
            armBottom.setPower(armBottomPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("This works", "HAHAHAHAHAHA!");
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();

        }
    }

    private void depositMineral(DcMotor bottom, DcMotor top) {

    }
}