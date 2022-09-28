package org.firstinspires.ftc.teamcode.NONAME;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ArmRef;
import org.firstinspires.ftc.teamcode.HardwareReference;

//NOTREADY <- you may see comments like this in the code. These represent code that would cause errors now that needs a specific fix. For example, the spinner isn't defined on or attached to the robot, so I leave it off for now.
@SuppressWarnings("FieldCanBeLocal")
@TeleOp(name="AUTOPLEASEWORK",group="Linear")
@Disabled
public class NONAME_Auto_CustomTest extends LinearOpMode {
    //Variable Initializations
    //Note: anything involving ArmRef.<something> is pulling variables from the ArmRef code. I put those variables there for ease of access, and also to allow FTCDashboard to access them.
    private ElapsedTime runtime = new ElapsedTime(); //Keeps track of the time during the code.
    private ElapsedTime instTimer= new ElapsedTime();
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
    private String[] commands = {"left","forward","posArm"};
    private double[] values = {0.0,0.1,520};
    private double[] times = {10000,1000,1000};
    private int[] encodes = {0,0,0};
    private int instNum=0;
    private int numcommands = 3;
    private String inst;
    private boolean autoArm = false; //This is a debug variable, if it is false, then the arm motor shuts off, allowing for manual repositioning of the arm.
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

    @Override
    public void runOpMode() {

        ldrive  = hardwareMap.get(DcMotor.class, HardwareReference.LEFT_DRIVE); //Initializing all the motors with the names from the HardwareReference.
        rdrive = hardwareMap.get(DcMotor.class, HardwareReference.RIGHT_DRIVE);
        arm = hardwareMap.get(DcMotor.class, HardwareReference.ARM);
        drum = hardwareMap.get(DcMotor.class, HardwareReference.BAND_DRUM);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        drum.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinner = hardwareMap.get(DcMotor.class, "spinner");


        ldrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
        instNum=0;
        instTimer.reset();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double leftPower=0;
            double rightPower=0;
            double spinnerPower=0;
            // Setup a variable for each drive wheel
            double drumPower=0; //This makes the drum controlled by the triggers.
            int curEncode = encodes[instNum];
            int thisEncode = ldrive.getCurrentPosition();
            boolean gotwo=curEncode<thisEncode;
            double curTime = times[instNum];
            double thisTime=instTimer.milliseconds();
            boolean go=curTime<thisTime;
            if (curEncode>0) {
                if (!(ldrive.isBusy())) {
                    instNum++;
                    if (instNum>=numcommands) {
                        return;
                    }
                    ldrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    ldrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    instTimer.reset();
                }
            } else {
                if (go) {
                    instNum++;
                    if (instNum>=numcommands) {
                        return;
                    }
                    ldrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    ldrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    instTimer.reset();
                }
            }
            curEncode = encodes[instNum];
            if (curEncode>0) {
                ldrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ldrive.setTargetPosition(curEncode);
            }
            String curInst=commands[instNum];
            double curValue=values[instNum];
            switch (curInst) {
                default:
                    break;
                case "forward":
                    leftPower=curValue;
                    rightPower=curValue;
                    break;
                case "backward":
                    leftPower=-curValue;
                    rightPower=curValue;
                    break;
                case "left":
                    leftPower=curValue;
                    rightPower=-curValue;
                    break;
                case "right":
                    leftPower=-curValue;
                    rightPower=curValue;
                    break;
                case "posArm":
                    targetArmPos=curValue;
                    break;
                case "setDrum":
                    drumPower=curValue;
                    break;
            }

            double armPower=0;
            armPowAdjust=ArmRef.VelocityMode.armPowAdjust;
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


            spinner.setPower(Range.clip(spinnerPower, -1.0, 1.0));
            //Setting all the motors' powers based on earlier variables.
            ldrive.setPower(Range.clip((leftPower), -1.0, 1.0));
            rdrive.setPower(Range.clip((rightPower), -1.0, 1.0));
            drum.setPower(Range.clip(drumPower,-1.0,1.0));
            telemetry.addData("Arm Pos", arm.getCurrentPosition());
            telemetry.addData("curInst", curInst);
            telemetry.addData("instNum", instNum);
            telemetry.addData("curTime", curTime);
            telemetry.addData("curEncode",curEncode);
            telemetry.addData("motorPos",ldrive.getCurrentPosition());
            telemetry.addData("time", instTimer.milliseconds());
            telemetry.update();
            //Sending telemetry feedback back to Dashboard.
            //int armPos=arm.getCurrentPosition();
            pastPos=thisPos;
            pastTime=thisTime;
            pastVel=velocity;
        }
    }
}