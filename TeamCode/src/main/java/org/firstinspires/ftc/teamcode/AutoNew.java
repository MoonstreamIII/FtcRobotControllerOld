package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//NOTREADY <- you may see comments like this in the code. These represent code that would cause errors now that needs a specific fix. For example, the spinner isn't defined on or attached to the robot, so I leave it off for now.
@SuppressWarnings("FieldCanBeLocal")
@Autonomous(name="Auto New",group="Linear")
//@Disabled
public class AutoNew extends LinearOpMode {
    //Variable Initializations
    //Note: anything involving ArmRef.<something> is pulling variables from the ArmRef code. I put those variables there for ease of access, and also to allow FTCDashboard to access them.
    private ElapsedTime runtime = new ElapsedTime(); //Keeps track of the time during the code.
    private ElapsedTime instTimer= new ElapsedTime();
    private DcMotor lfd = null; //Left tread motor
    private DcMotor rfd = null;  //Right tread motor
    private DcMotor lbd = null; //Left tread motor
    private DcMotor rbd = null;  //Right tread motor
    private DcMotor liftL = null; //Drum motor
    private DcMotor liftR = null; //Arm motor
    private Servo grabber = null;
    private double grabPos = 0.0;
    private double relPos = 0.0;
    private String[] commands = {"wait","lift","wait","forward","wait","strafeR","forward","lift"};
    private double[] values = {0,0.01,0,0.7,0,0.7,0.7,0.5};
    private double[] times = {100,0,1000,0,1000,0,0,0};
    private int[] encodes = {0,1660,0,100,0,5000,5000,0};
    private int instNum=0;
    private int numcommands = 8;
    private String inst;
    private double targetArmPos=0; //Target arm position, in encoder pulses.


    public void doCommand(int curEncode,double curValue, String curInst) {
        lfd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rfd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lbd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rbd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lfd.setPower(0);
        rfd.setPower(0);
        lbd.setPower(0);
        rbd.setPower(0);
        liftL.setPower(0);
        liftR.setPower(0);
        switch (curInst) {
            default:
                break;
            case "forward":
                lfd.setTargetPosition(curEncode);
                rfd.setTargetPosition(curEncode);
                lbd.setTargetPosition(curEncode);
                rbd.setTargetPosition(curEncode);
                lfd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rfd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lbd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rbd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lfd.setPower(curValue);
                rfd.setPower(curValue);
                lbd.setPower(curValue);
                rbd.setPower(curValue);
                break;
            case "backward":
                lfd.setTargetPosition(-curEncode);
                rfd.setTargetPosition(-curEncode);
                lbd.setTargetPosition(-curEncode);
                rbd.setTargetPosition(-curEncode);
                lfd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rfd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lbd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rbd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lfd.setPower(curValue);
                rfd.setPower(curValue);
                lbd.setPower(curValue);
                rbd.setPower(curValue);
                break;
            case "left":
                lfd.setTargetPosition(-curEncode);
                rfd.setTargetPosition(curEncode);
                lbd.setTargetPosition(-curEncode);
                rbd.setTargetPosition(curEncode);
                lfd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rbd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lbd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rfd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lfd.setPower(curValue);
                rfd.setPower(curValue);
                lbd.setPower(curValue);
                rbd.setPower(curValue);
                break;
            case "right":
                lfd.setTargetPosition(curEncode);
                rfd.setTargetPosition(-curEncode);
                lbd.setTargetPosition(curEncode);
                rbd.setTargetPosition(-curEncode);
                lfd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rfd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lbd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rbd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lfd.setPower(curValue);
                rfd.setPower(curValue);
                lbd.setPower(curValue);
                rbd.setPower(curValue);
                break;
            case "strafeL":
                lfd.setTargetPosition(-curEncode);
                rfd.setTargetPosition(curEncode);
                lbd.setTargetPosition(curEncode);
                rbd.setTargetPosition(-curEncode);
                lfd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rfd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lbd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rbd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lfd.setPower(curValue);
                rfd.setPower(curValue);
                lbd.setPower(curValue);
                rbd.setPower(curValue);
                break;
            case "strafeR":
                lfd.setTargetPosition(curEncode);
                rfd.setTargetPosition(-curEncode);
                lbd.setTargetPosition(-curEncode);
                rbd.setTargetPosition(curEncode);
                lfd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rfd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lbd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rbd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lfd.setPower(curValue);
                rfd.setPower(curValue);
                lbd.setPower(curValue);
                rbd.setPower(curValue);
                break;
            case "lift":
                liftL.setTargetPosition(curEncode);
                liftR.setTargetPosition(curEncode);
                liftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftL.setPower(curValue);
                liftR.setPower(curValue);
                break;
            case "grab":
                grabber.setPosition(grabPos);
                break;
            case "release":
                grabber.setPosition(relPos);
                break;
        }
    }
    @Override
    public void runOpMode() {

        lfd = hardwareMap.get(DcMotor.class, "lfd");
        rfd = hardwareMap.get(DcMotor.class, "rfd");
        lbd = hardwareMap.get(DcMotor.class, "lfd");
        rbd = hardwareMap.get(DcMotor.class, "rfd");
        liftL = hardwareMap.get(DcMotor.class, "liftL");
        liftR = hardwareMap.get(DcMotor.class, "liftR");
        grabber = hardwareMap.get(Servo.class, "grabber");


        lfd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rfd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lbd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rbd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lfd.setDirection(DcMotor.Direction.REVERSE); //Setting motor rotation directions and initializing them.
        rfd.setDirection(DcMotor.Direction.FORWARD);
        lbd.setDirection(DcMotor.Direction.REVERSE); //Setting motor rotation directions and initializing them.
        rbd.setDirection(DcMotor.Direction.FORWARD);
        liftL.setDirection(DcMotor.Direction.FORWARD); //Setting motor rotation directions and initializing them.
        liftR.setDirection(DcMotor.Direction.REVERSE);
        lfd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rfd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rbd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lbd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        instNum=0;
        instTimer.reset();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Setup a variable for each drive wheel
            int curEncode = encodes[instNum];
            double curTime = times[instNum];
            double thisTime=instTimer.milliseconds();
            boolean go=curTime<thisTime;
            if (curEncode==0) {
                if (go) {
                    instNum++;
                    if (instNum>=numcommands) {
                        return;
                    }
                    instTimer.reset();
                    curEncode = encodes[instNum];
                    String curInst=commands[instNum];
                    double curValue=values[instNum];
                    doCommand(curEncode,curValue,curInst);
                }
            } else {
                boolean motorsBusy = lfd.isBusy()||rfd.isBusy()||lbd.isBusy()||rbd.isBusy()||liftL.isBusy()||liftR.isBusy();
                if (!motorsBusy) {
                    instNum++;
                    if (instNum>=numcommands) {

                        return;
                    }
                    instTimer.reset();
                    curEncode = encodes[instNum];
                    String curInst=commands[instNum];
                    double curValue=values[instNum];
                    doCommand(curEncode,curValue,curInst);
                }
            }



            //Setting all the motors' powers based on earlier variables.
            telemetry.update();
            //Sending telemetry feedback back to Dashboard.
            //int armPos=arm.getCurrentPosition();
        }
    }
}