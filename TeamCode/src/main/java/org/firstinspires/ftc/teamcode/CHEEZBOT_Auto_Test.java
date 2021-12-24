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

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Disabled
@Autonomous(name="CHEEZBOT: Auto Test", group="Skybot")
public class CHEEZBOT_Auto_Test extends LinearOpMode {
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor lfd = null;
    private DcMotor rfd = null;
    private DcMotor lbd = null;
    private DcMotor rbd = null;
    private static final double TURN_SPEED    =AutoReference.power;
    private final double leg1 = AutoReference.leg1;
    private final double leg2 = AutoReference.leg2;
    private final double leg3 = AutoReference.leg3;
    private DcMotor ringBelt = null;
    private final double beltPwr=AutoReference.ringBeltPower;
    private final boolean debug=AutoReference.debug;

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        lfd  = hardwareMap.get(DcMotor.class, HardwareReference.LEFT_FRONT_DRIVE);
        rfd = hardwareMap.get(DcMotor.class, HardwareReference.RIGHT_FRONT_DRIVE);
        lbd  = hardwareMap.get(DcMotor.class, HardwareReference.LEFT_REAR_DRIVE);
        rbd = hardwareMap.get(DcMotor.class, HardwareReference.RIGHT_REAR_DRIVE);
        ringBelt = hardwareMap.get(DcMotor.class, HardwareReference.RING_BELT);
        lfd.setDirection(DcMotor.Direction.REVERSE);
        rfd.setDirection(DcMotor.Direction.FORWARD);
        lbd.setDirection(DcMotor.Direction.REVERSE);
        rbd.setDirection(DcMotor.Direction.FORWARD);
        lfd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rfd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lbd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rbd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lfd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rfd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lbd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rbd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */


        // Send telemetry message to signify robot waiting;
        dashboardTelemetry.addData("Status", "Ready to run");    //
        dashboardTelemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        lfd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lfd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Step 1:  Drive forward for 3 seconds
        if (debug) {
            while (true) {
                lfd.setPower(0.25);
                dashboardTelemetry.addData("Encoder Pos:", "" + lfd.getCurrentPosition());
                dashboardTelemetry.update();
            }
        } else {
        double cor = AutoReference.correction;
        lfd.setPower(TURN_SPEED);
        lbd.setPower(TURN_SPEED);
        rfd.setPower(TURN_SPEED*cor);
        rbd.setPower(TURN_SPEED*cor);
        runtime.reset();
        while (opModeIsActive() && (lfd.getCurrentPosition() < leg1)) {
            dashboardTelemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            dashboardTelemetry.update();
        }
        lfd.setPower(0);
        lbd.setPower(0);
        rfd.setPower(0);
        rbd.setPower(0);
        ringBelt.setPower(beltPwr);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < leg2)) {
            dashboardTelemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            dashboardTelemetry.update();
        }
        lfd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lfd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ringBelt.setPower(0);
        lfd.setPower(-TURN_SPEED);
        lbd.setPower(-TURN_SPEED);
        rfd.setPower(-TURN_SPEED*cor);
        rbd.setPower(-TURN_SPEED*cor);
        runtime.reset();
        while (opModeIsActive() && ((lfd.getCurrentPosition()) > leg3)) {
            dashboardTelemetry.addData("Path", ""+lfd.getCurrentPosition());
            dashboardTelemetry.update();
        }}

        // Step 4:  Stop and close the claw.
        lfd.setPower(0);
        rfd.setPower(0);
        lbd.setPower(0);
        rbd.setPower(0);

        dashboardTelemetry.addData("Path", "Complete");
        dashboardTelemetry.update();
        sleep(1000);
    }
}
