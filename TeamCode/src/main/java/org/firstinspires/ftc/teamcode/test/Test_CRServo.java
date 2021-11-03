
package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@SuppressWarnings("FieldCanBeLocal")
@TeleOp(name="Test CRServos",group="Test")
//@Disabled
public class Test_CRServo extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private CRServo s1 = null;
    private CRServo s2 = null;
    private CRServo s3 = null;
    private CRServo s4 = null;

    /*
     * CRServos in ports 0-3 must me configured as follows:
     *
     * CRServo in port 0: cs1
     * CRServo in port 1: cs2
     * CRServo in port 2: cs3
     * CRServo in port 3: cs4
     *
     * pressing the face buttons of the controller will set the crsevo to be powered:
     *
     * Y: CRServo in port 0
     * X: CRServo in port 1
     * B: CRServo in port 2
     * A: CRServo in port 3
     *
     * Moving the LEFT stick up or down will power the crservo forwards or backwards, respectively
     */

    private static final String CRSERVO_1 = "cs1";
    private static final String CRSERVO_2 = "cs2";
    private static final String CRSERVO_3 = "cs3";
    private static final String CRSERVO_4 = "cs4";

    private int activeServo = 0;

    private CRServo[] motors;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        s1 = hardwareMap.get(CRServo.class, CRSERVO_1);
        s2 = hardwareMap.get(CRServo.class, CRSERVO_2);
        s3 = hardwareMap.get(CRServo.class, CRSERVO_3);
        s4 = hardwareMap.get(CRServo.class, CRSERVO_4);

        motors = new CRServo[]{s1, s2, s3, s4};

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        s1.setDirection(CRServo.Direction.FORWARD);
        s2.setDirection(CRServo.Direction.FORWARD);
        s3.setDirection(CRServo.Direction.FORWARD);
        s4.setDirection(CRServo.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double power = gamepad1.left_stick_x;

            if(gamepad1.y) {
                activeServo = 0;
            }else if(gamepad1.x) {
                activeServo = 1;
            }else if(gamepad1.b) {
                activeServo = 2;
            }else if(gamepad1.a) {
                activeServo = 3;
            }

            (motors[activeServo]).setPower(power);


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("CRServo", "power (%.2f)", power);
            telemetry.update();
        }
    }
}
