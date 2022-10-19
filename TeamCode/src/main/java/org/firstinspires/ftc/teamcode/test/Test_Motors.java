
package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Disabled
@SuppressWarnings("FieldCanBeLocal")
@TeleOp(name="Test Motors",group="Test")
//@Disabled
public class Test_Motors extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor m1 = null;
    private DcMotor m2 = null;
    private DcMotor m3 = null;
    private DcMotor m4 = null;

    /*
    * Motors in ports 0-3 must me configured as follows:
    *
    * Motor in port 0: m1
    * Motor in port 1: m2
    * Motor in port 2: m3
    * Motor in port 3: m4
    *
    * pressing the face buttons of the controller will set the motor to be powered:
    *
    * Y: motor in port 0
    * X: motor in port 1
    * B: motor in port 2
    * A: motor in port 3
    *
    * Moving the LEFT stick up or down will power the motor forwards or backwards, respectively
    */

    private static final String MOTOR_1 = "m1";
    private static final String MOTOR_2 = "m2";
    private static final String MOTOR_3 = "m3";
    private static final String MOTOR_4 = "m4";

    private int activeMotor = 0;

    private DcMotor[] motors;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        m1 = hardwareMap.get(DcMotor.class, MOTOR_1);
        m2 = hardwareMap.get(DcMotor.class, MOTOR_2);
        m3 = hardwareMap.get(DcMotor.class, MOTOR_3);
        m4 = hardwareMap.get(DcMotor.class, MOTOR_4);

        motors = new DcMotor[]{m1,m2,m3,m4};

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        m1.setDirection(DcMotor.Direction.FORWARD);
        m2.setDirection(DcMotor.Direction.FORWARD);
        m3.setDirection(DcMotor.Direction.FORWARD);
        m4.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double power = -gamepad1.left_stick_y;

            if(gamepad1.y) {
                activeMotor = 0;
            }else if(gamepad1.x) {
                activeMotor = 1;
            }else if(gamepad1.b) {
                activeMotor = 2;
            }else if(gamepad1.a) {
                activeMotor = 3;
            }

            (motors[activeMotor]).setPower(power);


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motor", "power (%.2f)", power);
            telemetry.addData("activeMotor",""+activeMotor);
            telemetry.update();
        }
    }
}
