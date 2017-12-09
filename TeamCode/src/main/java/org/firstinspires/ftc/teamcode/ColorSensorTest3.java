package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by tonyp on 12/2/2017.
 */
@Autonomous
public class ColorSensorTest3 extends LinearOpMode {

    private DcMotor leftFrontDrive = null; //use left stick to go forward/back, use right stick to turn
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor strafeFrontDrive = null; //use bars
    private DcMotor strafeBackDrive = null;
    private DcMotor arm = null;
    private Servo leftClaw = null;
    private Servo rightClaw = null;
    private ColorSensor CSensor = null;
    private ElapsedTime runtime = new ElapsedTime();

    private static final double COUNTS_PER_MOTOR_REV = 140;
    private static final double WHEEL_DIAMETER_INCHES = 3;     // For figuring circumference
    private static final double COUNTS_PER_INCH = COUNTS_PER_MOTOR_REV / (WHEEL_DIAMETER_INCHES * 3.1415);
    private static final double SPEED = 0.1;
    private static final double DISTANCE1 = 36;
    private static final long STRAFE_TIME1 = 500;
    private static final long STRAFE_TIME2 = 500;

    private boolean ballDown = false;

    @Override
    public void runOpMode() throws InterruptedException {

        leftFrontDrive = hardwareMap.get(DcMotor.class, "LeftFrontDrive"); //initializes motors
        leftBackDrive = hardwareMap.get(DcMotor.class, "LeftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "RightBackDrive");
        strafeFrontDrive = hardwareMap.get(DcMotor.class, "StrafeFrontDrive");
        strafeBackDrive = hardwareMap.get(DcMotor.class, "StrafeBackDrive");
        arm = hardwareMap.get(DcMotor.class, "Arm");
        leftClaw = hardwareMap.get(Servo.class, "LeftClaw");
        rightClaw = hardwareMap.get(Servo.class, "RightClaw");
        CSensor = hardwareMap.get(ColorSensor.class, "ColorSensor");

        CSensor.enableLed(true);
        leftClaw.setPosition(0); //closes the claw with lb
        rightClaw.setPosition(0.5);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        //reverses direction of right motors, change if the direction is wrong

        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        strafeFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        strafeBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        strafeBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        strafeFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                leftFrontDrive.getCurrentPosition(),
                rightFrontDrive.getCurrentPosition(),
                leftBackDrive.getCurrentPosition(),
                rightBackDrive.getCurrentPosition());
        telemetry.update();

        // wait for the start button to be pressed.
        waitForStart();

        runtime.reset();
        if (opModeIsActive() && runtime.seconds() < 5) {
            encoderDrive(DISTANCE1, 5.0);  // S1: Forward 12 Inches with 5 Sec timeout
            sleep(500);
        }

        runtime.reset();
        while (opModeIsActive() && !ballDown && runtime.seconds() < 20) {
            if (CSensor.red() > CSensor.blue() && CSensor.red() > CSensor.green()) {
                rightClaw.setPosition(0);
                sleep(2000);
                ballDown=true;
            } else if (CSensor.blue() > CSensor.red() && CSensor.blue() > CSensor.green()) {
                leftClaw.setPosition(0.5);
                sleep(2000);
                ballDown=true;
            }
        }
    }

    public void encoderDrive(double distance, double timeOut) {
        int newLeftFrontTarget;
        int leftFrontPosition;
        int newLeftBackTarget;
        int leftBackPosition;
        int newRightFrontTarget;
        int rightFrontPosition;
        int newRightBackTarget;
        int rightBackPosition;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = leftFrontDrive.getCurrentPosition() + (int) Math.round((distance * COUNTS_PER_INCH));
            newLeftBackTarget = leftBackDrive.getCurrentPosition() + (int) Math.round((distance * COUNTS_PER_INCH));
            newRightFrontTarget = rightFrontDrive.getCurrentPosition() + (int) Math.round((distance * COUNTS_PER_INCH));
            newRightBackTarget = rightBackDrive.getCurrentPosition() + (int) Math.round((distance * COUNTS_PER_INCH));
            leftFrontDrive.setTargetPosition(newLeftFrontTarget);
            leftBackDrive.setTargetPosition(newLeftBackTarget);
            rightFrontDrive.setTargetPosition(newRightFrontTarget);
            rightBackDrive.setTargetPosition(newRightBackTarget);

            leftFrontPosition = leftFrontDrive.getCurrentPosition();
            leftBackPosition = leftBackDrive.getCurrentPosition();
            rightFrontPosition = rightFrontDrive.getCurrentPosition();
            rightBackPosition = rightBackDrive.getCurrentPosition();

            // Turn On RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            while (opModeIsActive() && (leftBackPosition < newLeftBackTarget) && (leftFrontPosition < newLeftFrontTarget)
                    && (rightBackPosition < newRightBackTarget) && (rightFrontPosition < newRightFrontTarget)
                    && (runtime.seconds() <= timeOut)) {
                leftFrontDrive.setPower(Math.abs(SPEED));
                leftBackDrive.setPower(Math.abs(SPEED));
                rightBackDrive.setPower(Math.abs(SPEED));
                rightFrontDrive.setPower(Math.abs(SPEED));

                leftFrontPosition = leftFrontDrive.getCurrentPosition();
                leftBackPosition = leftBackDrive.getCurrentPosition();
                rightFrontPosition = rightFrontDrive.getCurrentPosition();
                rightBackPosition = rightBackDrive.getCurrentPosition();
            }

            // Stop all motion;
            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);

            telemetry.addData("2 ", "motorFrontLeft:  " + String.format("%d", leftFrontDrive.getTargetPosition()));
            telemetry.addData("3 ", "motorFrontRight:  " + String.format("%d", rightFrontDrive.getTargetPosition()));
            telemetry.addData("4 ", "motorBackLeft:  " + String.format("%d", leftBackDrive.getTargetPosition()));
            telemetry.addData("5 ", "motorBackRight:  " + String.format("%d", rightBackDrive.getTargetPosition()));

            // Turn off RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(500);   // optional pause after each move
        }
    }

}