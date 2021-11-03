/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.SkyBot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutoReference;
import org.firstinspires.ftc.teamcode.HardwareReference;

@Disabled
@Autonomous(name="Skybot: Auto Under Bridge Far Right", group="Skybot")
public class SkyBotAuto_UnderBridge_FarRightSide extends LinearOpMode {
    /* Declare OpMode members. */
    private ElapsedTime     runtime = new ElapsedTime();
    private DcMotor lfd = null;
    private DcMotor rfd = null;
    private DcMotor lbd = null;
    private DcMotor rbd = null;
    private Servo leftHand = null;
    private Servo rightHand = null;
    static final double FORWARD_SPEED = AutoReference.FarBridge.power;
    private static final double TURN_SPEED    = AutoReference.FarBridge.power;
    private final double firstLegMultiplier = AutoReference.FarBridge.firstLegMultiplier;
    private final double leftFullOpen = 0.13;
    private final double rightFullOpen = 0.83;
    private final double rightOpen = 0.53;
    private final double leftOpen = 0.41;
    private final double leftClosed = 0.74;
    private final double rightClosed = 0.19;
    private final double leg1 = AutoReference.FarBridge.leg1;
    private final double leg2 = AutoReference.FarBridge.leg2;
    private final double leg3 = AutoReference.FarBridge.leg3;
    private DcMotor topSlide = null;
    private DcMotor bottomSlide = null;

    @Override
    public void runOpMode() {
        leftHand = hardwareMap.get(Servo.class, HardwareReference.LEFT_HAND_SERVO);
        rightHand = hardwareMap.get(Servo.class, HardwareReference.RIGHT_HAND_SERVO);
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        lfd  = hardwareMap.get(DcMotor.class, HardwareReference.LEFT_FRONT_DRIVE);
        rfd = hardwareMap.get(DcMotor.class, HardwareReference.RIGHT_FRONT_DRIVE);
        lbd  = hardwareMap.get(DcMotor.class, HardwareReference.LEFT_REAR_DRIVE);
        rbd = hardwareMap.get(DcMotor.class, HardwareReference.RIGHT_REAR_DRIVE);
        //foo = hardwareMap.get(DcMotor.class, "foo_motor");
        topSlide = hardwareMap.get(DcMotor.class, HardwareReference.LINEAR_SLIDE_TOP);
        bottomSlide = hardwareMap.get(DcMotor.class, HardwareReference.LINEAR_SLIDE_BOTTOM);
        lfd.setDirection(DcMotor.Direction.FORWARD);
        rfd.setDirection(DcMotor.Direction.REVERSE);
        lbd.setDirection(DcMotor.Direction.FORWARD);
        rbd.setDirection(DcMotor.Direction.REVERSE);
        topSlide.setDirection(DcMotor.Direction.FORWARD);
        bottomSlide.setDirection(DcMotor.Direction.REVERSE);
        leftHand.setDirection(Servo.Direction.FORWARD);
        rightHand.setDirection(Servo.Direction.FORWARD);

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        leftHand.setPosition(leftFullOpen);
        rightHand.setPosition(rightFullOpen);


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        while (opModeIsActive() && (runtime.seconds() < leg3)) {
            telemetry.addData("Path", "Leg 0: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way
        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way
        lfd.setPower(-TURN_SPEED*firstLegMultiplier);
        rfd.setPower(-TURN_SPEED*firstLegMultiplier);
        lbd.setPower(-TURN_SPEED*firstLegMultiplier);
        rbd.setPower(-TURN_SPEED*firstLegMultiplier);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < leg1)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        // Step 1:  Drive forward for 3 seconds
        lfd.setPower(-TURN_SPEED);
        rfd.setPower(TURN_SPEED);
        lbd.setPower(TURN_SPEED);
        rbd.setPower(-TURN_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < leg2)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 2:  Spin left 1.3 seconds
        /*lfd.setPower(-TURN_SPEED);
        rfd.setPower(TURN_SPEED);
        lbd.setPower(-TURN_SPEED);
        rbd.setPower(TURN_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < leg2)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 3:  Drive Forwards for 1 Second
        lfd.setPower(TURN_SPEED);
        rfd.setPower(TURN_SPEED);
        lbd.setPower(TURN_SPEED);
        rbd.setPower(TURN_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < leg3)) {
            telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }*/

        // Step 4:  Stop and close the claw.
        lfd.setPower(0);
        rfd.setPower(0);
        lbd.setPower(0);
        rbd.setPower(0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
