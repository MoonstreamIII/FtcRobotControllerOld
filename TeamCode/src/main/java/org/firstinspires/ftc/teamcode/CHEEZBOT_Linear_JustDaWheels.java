package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//This is a system where the triggers control the speed of the bot. For example, if you have the left stick at full forward and the left trigger half pressed in, you get half power.
@SuppressWarnings("FieldCanBeLocal")
@TeleOp(name="CHEEZBOT Op Mode - JustDaWheels",group="Linear")
@Disabled
public class CHEEZBOT_Linear_JustDaWheels extends LinearOpMode {

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
    //private DcMotor ringLift = null;
    //private Servo door = null;
    //Currently, all servo positions must remain on
    //the interval [0.13,0.87], or the servos will not respond.
    private final double lowPowerMod = 0.35;
    private final double highPowerMod = 0.7;
    private final double turnReductionLowPower = 0.80;
    private final double doorOpen=45.0; //Open position for the door
    private final double doorClosed=45.0; //Closed position for the door
    private final boolean openIsPositive=doorOpen>=doorClosed;
    private final double doorRate=0.1;//Rate of movement for the door
    private final double dr =openIsPositive?doorRate:-doorRate;
    private final double doorMin=Math.min(doorOpen,doorClosed);
    private final double doorMax=Math.max(doorOpen,doorClosed);
    private boolean rightStrafe = false;
    private double modulation = 0;
    private boolean lowPower = false;
    private double doorPos=doorClosed;

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
        //foo = hardwareMap.get(DcMotor.class, "foo_motor");
        //ringLift = hardwareMap.get(DcMotor.class, HardwareReference.RING_LIFT);
        //door = hardwareMap.get(Servo.class,HardwareReference.DOOR_SERVO);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        lfd.setDirection(DcMotor.Direction.FORWARD);
        rfd.setDirection(DcMotor.Direction.REVERSE);
        lbd.setDirection(DcMotor.Direction.FORWARD);
        rbd.setDirection(DcMotor.Direction.REVERSE);
        lfd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rfd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lbd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rbd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //door.setPosition(doorPos);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Setup a variable for each drive wheel to save power level for telemetry
            double doorChange;
            double leftPower;
            double rightPower;
            double liftPower;
            if (lowPower) {
                modulation = lowPowerMod;
            } else {
                modulation = highPowerMod;
            }

            //Toggles the function that, when active, makes the hand close when any of the bumpers or triggers are pressed. If this is off, the left bumper and trigger open the hand, and the right bumper and trigger close the hand.
            //Toggles if the linear slides are controlled seperately or together. If on, the left stick controls the lower slide and the right stick controls the right slide. If off, both sticks control both slides.
            if (gamepad1.left_bumper&&gamepad1.right_bumper&&gamepad1.left_stick_button) {
                rightStrafe = false;
            }
            if (gamepad1.left_bumper&&gamepad1.right_bumper&&gamepad1.right_stick_button) {
                rightStrafe = true;
            }
            //modulation = gamepad1.left_trigger+gamepad1.right_trigger;
            /*if (gamepad1.left_bumper||gamepad1.right_bumper) {
                modulation = 1;
            }*/
            if (gamepad1.left_bumper) {
                lowPower = true;
            }
            if (gamepad1.right_bumper) {
                lowPower = false;
            }
            if (gamepad2.right_bumper) {
                doorPos = doorOpen;
            }
            if (gamepad2.left_bumper) {
                doorPos = doorClosed;
            }
            doorChange=(-gamepad2.left_trigger*dr)+(gamepad2.right_trigger*dr);


            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            //NOTES: gamepad1.left_stick_y increases as stick goes down.
            double drive = -gamepad1.left_stick_y+ -gamepad1.right_stick_y;
            //double drive = Math.max( gamepad1.left_stick_y, Math.max(gamepad1.right_stick_y, gamepad1.right_trigger - gamepad1.left_trigger));
            double turn;
            double turnReduction = 1;
            if (lowPower) {
                turnReduction=turnReductionLowPower;
            }

            if (rightStrafe) {
                turn = -(gamepad1.left_stick_x*turnReduction);  //Turning using the left stick.
            } else {
                turn = -(gamepad1.right_stick_x*turnReduction);
            }
            double strafe;
            if (rightStrafe) {
                strafe = -(gamepad1.right_stick_x);  //Strafing using the right stick
            } else
                strafe = -(gamepad1.left_stick_x);  //Strafing using the right stick
            leftPower    = -(drive - turn);
            rightPower   = -(drive + turn);
            liftPower = (-gamepad2.right_stick_y);
            /*cTODO closed = (gamepad2.right_bumper||gamepad2.left_bumper);*/


            //Figuring out if the hand is open or closed

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels
            doorPos=Range.clip(doorPos+doorChange,doorMin,doorMax);
            lfd.setPower(Range.clip((leftPower-strafe)*modulation, -1.0, 1.0));
            rfd.setPower(Range.clip((rightPower+strafe)*modulation, -1.0, 1.0));
            lbd.setPower(Range.clip((leftPower+strafe)*modulation, -1.0, 1.0));
            rbd.setPower(Range.clip((rightPower-strafe)*modulation, -1.0, 1.0));
            //ringLift.setPower(Range.clip(liftPower,-1.0,1.0));
            //door.setPosition(doorPos);



        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        telemetry.update();
        }
    }
}