package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

//This is a system where the triggers control the speed of the bot. For example, if you have the left stick at full forward and the left trigger half pressed in, you get half power.
@SuppressWarnings("FieldCanBeLocal")
@TeleOp(name="NONAME Op Mode - Arm Velocity Control",group="Linear")
//@Disabled
public class NONAME_Linear_ArmVelocity extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    //ldrive is left front drive
    //rdrive is right front drive
    //lbd is left back drive
    //rbd is right back drive
    private DcMotor ldrive = null;
    private DcMotor rdrive = null;
    private DcMotor drum = null;
    private DcMotor arm=null;
    //private DcMotor ringLift = null;
    //private Servo door = null;
    //Currently, all servo positions must remain on
    //the interval [0.13,0.87], or the servos will not respond.
    private double pastTime = 0.0;
    private double pastPos = 0.0;
    private double thisTime= 0.0;
    private double thisPos=0.0;
    private final double lowPowerMod = 0.35;
    private final double highPowerMod = 0.7;
    private final double turnReductionLowPower = 0.80;
    private double armPosA=ArmRef.posA;
    private double armPosB=ArmRef.posB;
    private double armPosX=ArmRef.posX;
    private double speedDistMultiplier=ArmRef.SpeedMode.speedDistMultiplier;
    private double powerChangeMultiplier=ArmRef.SpeedMode.powerChangeMultiplier;
    private double armPowerLimit=ArmRef.SpeedMode.armPowerLimit;
    private double baseArmPower=ArmRef.SpeedMode.baseArmPower;
    private double baseArmSpeed=ArmRef.SpeedMode.baseArmSpeed;
    private  double armPosY=ArmRef.posY;
    private boolean autoArm=true;
    private double targetArmPos=0;
    private boolean rightStrafe = false;
    private double modulation = 0;
    private boolean lowPower = false;
    private boolean xlock=false;

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        //telemetry.addData("Status", "Initialized");
        //telemetry.update();
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        ldrive  = hardwareMap.get(DcMotor.class, HardwareReference.LEFT_DRIVE);
        rdrive = hardwareMap.get(DcMotor.class, HardwareReference.RIGHT_DRIVE);
        arm = hardwareMap.get(DcMotor.class, HardwareReference.ARM);
        drum = hardwareMap.get(DcMotor.class, HardwareReference.BAND_DRUM);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        drum.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //foo = hardwareMap.get(DcMotor.class, "foo_motor");
        //ringLift = hardwareMap.get(DcMotor.class, HardwareReference.RING_LIFT);
        //door = hardwareMap.get(Servo.class,HardwareReference.DOOR_SERVO);

        // Most roobts need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        ldrive.setDirection(DcMotor.Direction.REVERSE);
        rdrive.setDirection(DcMotor.Direction.FORWARD);
        ldrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setDirection(DcMotor.Direction.REVERSE);
        drum.setDirection(DcMotor.Direction.FORWARD);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drum.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pastTime=runtime.milliseconds();
        pastPos=arm.getCurrentPosition();
        //doro.setPosition(doorPos);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;
            if (lowPower) {
                modulation = lowPowerMod;
            } else {
                modulation = highPowerMod;
            }

            //Toggles the funtcion that, when active, makes the hand close when any of the bumpers or triggers are pressed. If this is off, the left bumper and trigger open the hand, and the right bumper and trigger close the hand.
            //Toggles if the linear slides are controlled seperately or together. If on, the left stick controls the lower slide and the right stick controls the right slide. If off, both sticks control both slides.
            if (gamepad1.left_bumper&&gamepad1.right_bumper&&gamepad1.left_stick_button) {
                rightStrafe = false;
            }
            if (gamepad1.left_bumper&&gamepad1.right_bumper&&gamepad1.right_stick_button) {
                rightStrafe = true;
            }
            //modluation = gamepad1.left_trigger+gamepad1.right_trigger;
            /*if (gamepad1.left_bumper||gamepad1.right_bumper) {
                modulation = 1;
            }*/
            if (gamepad2.x&&!xlock) {
                xlock=true;
                autoArm=!autoArm;
            }
            if (!gamepad2.x) {
                xlock=false;
            }

            armPosA=ArmRef.posA;
            armPosB=ArmRef.posB;
            armPosX=ArmRef.posX;
            armPosY=ArmRef.posY;
            speedDistMultiplier=ArmRef.SpeedMode.speedDistMultiplier;
            powerChangeMultiplier=ArmRef.SpeedMode.powerChangeMultiplier;
            armPowerLimit=ArmRef.SpeedMode.armPowerLimit;
            baseArmPower=ArmRef.SpeedMode.baseArmPower;
            baseArmSpeed=ArmRef.SpeedMode.baseArmSpeed;
            if (gamepad1.left_bumper) {
                lowPower = true;
            }
            if (gamepad1.right_bumper) {
                lowPower = false;
            }
            if (gamepad1.a&&!gamepad1.start) {
                targetArmPos = armPosA;
            }
            if (gamepad1.b&&!gamepad1.start) {
                targetArmPos = armPosB;
            }
            if (gamepad1.x&&!gamepad1.start) {
                targetArmPos = armPosX;
            }
            if (gamepad1.y&&!gamepad1.start) {
                targetArmPos = armPosY;
            }
            if (gamepad1.left_bumper&&gamepad1.right_bumper) {
                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            double drumPower=0;
            drumPower+=gamepad1.right_trigger;
            drumPower-=gamepad1.left_trigger;

            if (autoArm) {
                arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } else {
                arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
            // Choose to drive using either Tnak Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses baisc math to combine motions and is easier to drive straight.
            //NOTES: gamepad1.left_stick_y increases as stick goes down.
            double drive = -gamepad1.left_stick_y;
            //double drive = Math.max( gamepad1.left_stick_y, Math.max(gamepad1.right_stick_y, gamepad1.right_trigger - gamepad1.left_trigger));
            double turn;
            double turnReduction = 1;
            if (lowPower) {
                turnReduction=turnReductionLowPower;
            }

            turn = (gamepad1.left_stick_x*turnReduction);  //Tunring using the left stick.

            leftPower    = -(drive - turn);
            rightPower   = -(drive + turn);
            /*TDOO clodse = (gamepad2.right_bumper||gamepad2.left_bumper);*/


            //Figuring out if the hand is open or closed

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels
            //ldrive.setPower(1);
            //rdrive.setPower(-1);
            if (targetArmPos<0) {targetArmPos=0;}
            targetArmPos+=gamepad1.right_stick_y;
            /*double armPower=0;
            double distance=((targetArmPos-arm.getCurrentPosition())/armDistMultiplier)*tgtArmPower;
            if (autoArm) {
                arm.setPower(Range.clip(distance, -armPowerLimit, armPowerLimit));
            } else {
                arm.setPower(0);
            }*/
            int armPos=arm.getCurrentPosition();
           ldrive.setPower(Range.clip((leftPower)*modulation, -1.0, 1.0));
           rdrive.setPower(Range.clip((rightPower)*modulation, -1.0, 1.0));
           drum.setPower(Range.clip(drumPower,-1.0,1.0));
            dashboardTelemetry.addData("Arm", "Position: "+armPos);
            dashboardTelemetry.addData("Arm2", "Target Position: "+targetArmPos);
            //dashboardTelemetry.addData("Arm Power:",""+distance);
            dashboardTelemetry.addData("AutoArm", "State: "+(autoArm?"True":"False"));
            dashboardTelemetry.update();
            //ringLift.setPower(Range.clip(liftPower,-1.0,1.0));
            //door.setPosition(doorPos);



        // Show the elapsed game time and wheel power.
        //telemetry.addData("Status", "Run Time: " + runtime.toString());
        //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        //telemetry.update();
        }
    }
}