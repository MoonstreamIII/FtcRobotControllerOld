package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//NOTREADY <- you may see comments like this in the code. These represent code that would cause errors now that needs a specific fix. For example, the spinner isn't defined on or attached to the robot, so I leave it off for now.
@SuppressWarnings("FieldCanBeLocal")
@TeleOp(name="NONAME Op Mode - Arm Velocity Control Telemetry Spinners",group="Linear")
//@Disabled
public class NONAME_Linear_ArmGravityTelemSpinners extends LinearOpMode {
    //Variable Initializations
    //Note: anything involving ArmRef.<something> is pulling variables from the ArmRef code. I put those variables there for ease of access, and also to allow FTCDashboard to access them.
    private ElapsedTime runtime = new ElapsedTime(); //Keeps track of the time during the code.
    private DcMotor ldrive = null; //Left tread motor
    private DcMotor rdrive = null;  //Right tread motor
    private DcMotor drum = null; //Drum motor
    private DcMotor arm=null; //Arm motor
   private DcMotor spinner=null; //Carousel spinner motor
    private double pastTime = 0.0;
    private int pastPos = 0;
    private double pastVel=0;
    private double thisTime= 0.0;
    private int thisPos=0;
    private double armPosA=ArmRef.posA; //Setting arm positions based on the ArmRef class (I set them there mostly for ease of access and also because that's where FTCDashboard can access them)
    private double armPosB=ArmRef.posB;
    private double armPosX=ArmRef.posX;
    private double armPosY=ArmRef.posY;
    private boolean autoArm=ArmRef.autoArm; //This is a debug variable, if it is false, then the arm motor shuts off, allowing for manual repositioning of the arm.
    private double armVelocitySlope = ArmRef.VelocityMode.armVelocitySlope;
    private double armPowerSlope = ArmRef.VelocityMode.armPowerSlope;
    private double armPowerLimit= ArmRef.VelocityMode.armPowerLimit;
    private double armVelocityLimit = ArmRef.VelocityMode.armVelocityLimit;
    private double armInterp = ArmRef.VelocityMode.armInterp;
    private double armVelInterp = ArmRef.VelocityMode.armVelInterp;
    private double armPowAdjust = ArmRef.VelocityMode.armPowAdjust;
    private double targVelInterp=0.0;
    private double targPowerInterp=0.0;
    public int zeroPos=ArmRef.GravMode.zeroPos;
    public double gravPow=ArmRef.GravMode.gravPow;
    public double sidewaysDist =ArmRef.GravMode.sidewaysDist;
    public double sidewaysPos=zeroPos- sidewaysDist;
    private double targetArmPos=0; //Target arm position, in encoder pulses.
    private double modulation = 0.7; //Multiplier for how much the motor power should be reduced.
    private boolean xlock=false; //This is a variable used to detect if X is being held, explained later.

    @Override
    public void runOpMode() {
        //FtcDashboard dashboard = FtcDashboard.getInstance(); //Initializing Dashboard
        //  Telemetry dashboardTelemetry = dashboard.getTelemetry();

        ldrive  = hardwareMap.get(DcMotor.class, HardwareReference.LEFT_DRIVE); //Initializing all the motors with the names from the HardwareReference.
        rdrive = hardwareMap.get(DcMotor.class, HardwareReference.RIGHT_DRIVE);
        arm = hardwareMap.get(DcMotor.class, HardwareReference.ARM);
        drum = hardwareMap.get(DcMotor.class, HardwareReference.BAND_DRUM);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        drum.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinner = hardwareMap.get(DcMotor.class, "spinner");


        ldrive.setDirection(DcMotor.Direction.FORWARD); //Setting motor rotation directions and initializing them.
        rdrive.setDirection(DcMotor.Direction.REVERSE);
        ldrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setDirection(DcMotor.Direction.REVERSE);
        drum.setDirection(DcMotor.Direction.FORWARD);
        drum.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spinner.setDirection(DcMotor.Direction.FORWARD);
        spinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //This sets the arm's position to 0, to set the origin.
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pastTime=runtime.milliseconds();
        pastPos=0;
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Setup a variable for each drive wheel
            double leftPower;
            double rightPower;
            if (gamepad2.x&&!xlock&&!gamepad2.start) { //This locks/unlocks the arm. xlock is used to keep autoArm from being toggled every update.
                xlock=true;
                autoArm=!autoArm;
            }
            if (!gamepad2.x&&!gamepad2.start) {
                xlock=false;
            }
            if (gamepad1.a&&!gamepad1.start) { //If A is pressed, set the target arm position to the preset for A.
                targetArmPos = armPosA;
            }
            if (gamepad1.b&&!gamepad1.start) { //If B is pressed, set the target arm position to the preset for B.
                targetArmPos = armPosB;
            }
            if (gamepad1.x&&!gamepad1.start) { //If X is pressed, set the target arm position to the preset for X.
                targetArmPos = armPosX;
            }
            if (gamepad1.y&&!gamepad1.start) { //If Y is pressed, set the target arm position to the preset for Y.
                targetArmPos = armPosY;
            }
            /*if (gamepad1.left_bumper&&gamepad1.right_bumper) { //If both bumpers are pressed, zero the arm positions again.
                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }*/
            double drumPower=0; //This makes the drum controlled by the triggers.
            drumPower+=gamepad1.right_trigger;
            drumPower-=gamepad1.left_trigger;

            if (autoArm) { //If autoArm is on, but the arm is set to 0 power, have the motor hold its position. If autoArm is not on, have the motor not hold its position so it can be manually readjusted.
                arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } else {
                arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }

            //This runs the calculations for motor power based on stick position.
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.left_stick_x;
            leftPower    = (drive + turn);
            rightPower   = (drive - turn);

            //This prevents the arm from thinking it should go farther down than it can, to prevent it from pressing into the control hub.
            if (targetArmPos<0) {
                targetArmPos=0;
            }

            double armPowAdjustMul=1.0;

            //This calculates the target power+direction that should be delivered to the arm motor. I'm currently in the process of writing a better version of the arm control code.
            if (gamepad1.right_stick_button) {
                armPowAdjustMul=2.0;
            }
            double armPower=0;
            targetArmPos+=gamepad1.right_stick_y; //Allows for fine control of arm position using the right stick.
            armPowAdjust=ArmRef.VelocityMode.armPowAdjust*armPowAdjustMul;
            double posDiff=targetArmPos-arm.getCurrentPosition();
            double targetArmVelocity=(Range.clip(posDiff/armVelocitySlope,-1.0,1.0)*armVelocityLimit);
            targVelInterp+=(targetArmVelocity-targVelInterp)*armVelInterp;
            //Calculate the current velocity of the arm (calculated in encoder positions/ms)
            thisPos=arm.getCurrentPosition();
            thisTime=runtime.milliseconds();
            double armAngRad=Math.cos(((arm.getCurrentPosition()-sidewaysPos)*Math.PI)/(sidewaysDist*2));
            double armGravComp=armAngRad*gravPow;
            double timeChange=thisTime-pastTime;
            double posChange=thisPos-pastPos;
            double armVelocity=posChange/timeChange;
            double velocity=pastVel+((armVelocity-pastVel)*armInterp);
            double velDiff=targVelInterp-velocity;
            if (autoArm) { //If autoArm is on, but the arm is set to 0 power, have the motor hold its position. If autoArm is not on, have the motor not hold its position so it can be manually readjusted.
                arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            } else {
                arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
            double targetArmPower=Range.clip(velDiff/armPowerSlope,-1.0,1.0);
            targPowerInterp+=targetArmPower*armPowAdjust;
            targPowerInterp=Range.clip(targPowerInterp,-armPowerLimit,armPowerLimit);
            if (autoArm) {
                armPower=targPowerInterp+armGravComp;
                arm.setPower(Range.clip(armPower, -1.0, 1.0));
            } else {
                arm.setPower(0);
            }
            //Set the spinner to be controlled by the bumpers.
            double spinnerPower=0;
            if (gamepad1.left_bumper) {
                spinnerPower=-1;
            }
            if (gamepad1.right_bumper) {
                spinnerPower=1;
            }
            spinner.setPower(Range.clip(spinnerPower, -1.0, 1.0));

            //Setting all the motors' powers based on earlier variables.
            ldrive.setPower(Range.clip((leftPower)*modulation, -1.0, 1.0));
            rdrive.setPower(Range.clip((rightPower)*modulation, -1.0, 1.0));
            drum.setPower(Range.clip(drumPower,-1.0,1.0));

            //Sending telemetry feedback back to Dashboard.
            int armPos=arm.getCurrentPosition();
            telemetry.addData("Arm Pos", arm.getCurrentPosition());
            telemetry.addData("Target Arm Pos", targetArmPos);
            telemetry.addData("Arm Pos Change", posChange);
            telemetry.addData("Arm Pos Diff", posDiff);
            telemetry.addData("Time Diff", timeChange);
            telemetry.addData("Target Arm Power",targetArmPower);
            telemetry.addData("Target Arm Velocity", targetArmVelocity);
            telemetry.addData("Arm Velocity", armVelocity);
            telemetry.addData("Arm Velocity (Interpolated)",velocity);
            telemetry.addData("Target Arm Velocity (Interpolated)",targVelInterp);
            telemetry.addData("Target Arm Power (Interpolated)",targPowerInterp);
            telemetry.addData("AutoArm", "State: "+(autoArm?"True":"False"));
            telemetry.addData("Arm Gravity Compensation",armGravComp);
            telemetry.addData("Arm Angle (Radians)",armAngRad);
            telemetry.addData("armPower",armPower);
            telemetry.update();
            pastPos=thisPos;
            pastTime=thisTime;
            pastVel=velocity;
        }
    }
}