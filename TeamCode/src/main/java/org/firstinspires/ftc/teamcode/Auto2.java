
package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/*
 * Test the dashboard gamepad integration.
 */
//@Disabled

@Autonomous(name="AutoTest 2", group="Skybot")
    public class Auto2 extends LinearOpMode {
        private ElapsedTime runtime = new ElapsedTime();
        //front right
        private DcMotor motorFR = null;
        //back right
        private DcMotor motorBR = null;
        //front left
        private DcMotor motorFL = null;
        //back left
        private DcMotor motorBL = null;
        private Servo grabber = null;
        private DcMotor liftL = null;
        private DcMotor liftR = null;
        private double relPos = 0;
        private double grabPos = 0.25;
        private double pos = 0;
        //potentially reduce power, not written yet
        private int leg1enc = 2000;
        private double motorPower=0.75;
        private double liftPower=0.75;
        private int posX = 1110;//680=lowest  1108=middle 1660=top
        private int posY = 1660;
        private int posA = 150;
        private int posB = 680;

        @Override
        public void runOpMode() {
            //set up front right
            motorFR  = hardwareMap.get(DcMotor.class,  "rfd");
            motorFR.setDirection(DcMotor.Direction.REVERSE);
            //set up back right
            motorBR = hardwareMap.get(DcMotor.class, "rbd");
            motorBR.setDirection(DcMotor.Direction.REVERSE);
            //set up front left
            motorFL = hardwareMap.get(DcMotor.class, "lfd");
            motorFL.setDirection(DcMotor.Direction.FORWARD);
            //set up back left
            motorBL = hardwareMap.get(DcMotor.class, "lbd");
            motorBL.setDirection(DcMotor.Direction.FORWARD);

            liftL = hardwareMap.get(DcMotor.class, "liftL");
            liftL.setDirection(DcMotor.Direction.REVERSE);
            liftR = hardwareMap.get(DcMotor.class, "liftR");
            liftR.setDirection(DcMotor.Direction.FORWARD);
            grabber = hardwareMap.get(Servo.class, "grabber");
            motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftL.setTargetPosition(0);
            liftR.setTargetPosition(0);
            liftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            waitForStart();
            while(opModeIsActive()){
                runtime.reset();
                while (runtime.seconds()<5) {}
                motorFR.setPower(motorPower);
                motorBR.setPower(motorPower);
                motorFL.setPower(motorPower);
                motorBL.setPower(motorPower);
                liftL.setPower(liftPower);
                liftR.setPower(liftPower);
                liftL.setTargetPosition(150);
                liftR.setTargetPosition(150);
                motorFR.setTargetPosition(-leg1enc);
                motorBR.setTargetPosition(-leg1enc);
                motorFL.setTargetPosition(-leg1enc);
                motorBL.setTargetPosition(-leg1enc);
                while (motorFL.isBusy()) {}
                liftL.setTargetPosition(0);
                liftR.setTargetPosition(0);
                while (liftL.isBusy()||liftR.isBusy()) {}
            }

        }
    }