
package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@SuppressWarnings("FieldCanBeLocal")
@TeleOp(name="Test Servos",group="Test")
//@Disabled
public class Test_Servo extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Servo s1 = null;
    private Servo s2 = null;
    private Servo s3 = null;
    private Servo s4 = null;

    /*
     * Servos in ports 0-3 must me configured as follows:
     *
     * Servo in port 0: s1
     * Servo in port 1: s2
     * Servo in port 2: s3
     * Servo in port 3: s4
     *
     * pressing the face buttons of the controller will set the Servo to be powered:
     *
     * Y: Servo in port 0
     * X: Servo in port 1
     * B: Servo in port 2
     * A: Servo in port 3
     *
     * Moving the LEFT stick left or right will move the servo backwards or forwards, respectively
     */

    private static final String SERVO_1 = "s1";
    private static final String SERVO_2 = "s2";
    private static final String SERVO_3 = "s3";
    private static final String SERVO_4 = "s4";

    private int activeServo = 0;

    private Servo[] motors;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        s1 = hardwareMap.get(Servo.class, SERVO_1);
        s2 = hardwareMap.get(Servo.class, SERVO_2);
        s3 = hardwareMap.get(Servo.class, SERVO_3);
        s4 = hardwareMap.get(Servo.class, SERVO_4);

        motors = new Servo[]{s1, s2, s3, s4};

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        s1.setDirection(Servo.Direction.FORWARD);
        s2.setDirection(Servo.Direction.FORWARD);
        s3.setDirection(Servo.Direction.FORWARD);
        s4.setDirection(Servo.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double pos = gamepad1.left_stick_x;

            if(gamepad1.y) {
                activeServo = 0;
            }else if(gamepad1.x) {
                activeServo = 1;
            }else if(gamepad1.b) {
                activeServo = 2;
            }else if(gamepad1.a) {
                activeServo = 3;
            }

            (motors[activeServo]).setPosition(pos);


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Servo", "pos (%.2f)", pos);
            telemetry.addData("ActiveServo", ""+activeServo);
            telemetry.update();
        }
    }
}
