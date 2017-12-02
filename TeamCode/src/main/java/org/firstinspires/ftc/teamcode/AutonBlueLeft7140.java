package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by tonyp on 10/28/2017.
 */
@Autonomous(name = "AutonBlueLeft7140", group = "Autonomous")
@Disabled
public class AutonBlueLeft7140 extends LinearOpMode {

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
    private Servo jewelArm = null;

    private ElapsedTime runtime = new ElapsedTime();
    private static final int PULSES_PER_MOTOR_REV = 7;
    private static final int MOTOR_GEAR = 20;
    private static final double COUNTS_PER_MOTOR_REV = PULSES_PER_MOTOR_REV * MOTOR_GEAR;
    private static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    private static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    private static final double SPEED = 0.4;
    private static final long STRAFE_TIME1 = 500;
    private static final long STRAFE_TIME2 = 2000;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        leftFrontDrive = hardwareMap.get(DcMotor.class, "LeftFrontDrive"); //initializes motors
        leftBackDrive = hardwareMap.get(DcMotor.class, "LeftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "RightBackDrive");
        strafeFrontDrive = hardwareMap.get(DcMotor.class, "StrafeFrontDrive");
        strafeBackDrive = hardwareMap.get(DcMotor.class, "StrafeBackDrive");
        arm = hardwareMap.get(DcMotor.class, "Arm");
        leftClaw = hardwareMap.get(Servo.class, "LeftClaw");
        rightClaw = hardwareMap.get(Servo.class, "RightClaw");
        jewelArm = hardwareMap.get(Servo.class, "JewelArm");
        CSensor = hardwareMap.get(ColorSensor.class, "ColorSensor");

        // Set the LED in the beginning
        CSensor.enableLed(true);

        leftClaw.setPosition(0); //closes the claw with lb
        rightClaw.setPosition(0.5);

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        strafeFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        strafeBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Turn On RUN_TO_POSITION
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData("Status", "Initialized");

        waitForStart();
        runtime.reset();

        encoderDrive(SPEED, 12, 12, 5.0);  // S1: Forward 12 Inches with 5 Sec timeout

        if (CSensor.blue() > CSensor.red() && CSensor.blue() > CSensor.green()) {
            strafeBackDrive.setPower(SPEED);
            strafeFrontDrive.setPower(SPEED);
            sleep(STRAFE_TIME1);
            strafeBackDrive.setPower(0);
            strafeFrontDrive.setPower(0);
        }
        jewelArm.setPosition(0.75);
        strafeBackDrive.setPower(SPEED);
        strafeFrontDrive.setPower(SPEED);
        sleep(STRAFE_TIME2);
        strafeBackDrive.setPower(0);
        strafeFrontDrive.setPower(0);

    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftFrontDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = rightFrontDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            leftFrontDrive.setTargetPosition(newLeftTarget);
            leftBackDrive.setTargetPosition(newLeftTarget);
            rightFrontDrive.setTargetPosition(newRightTarget);
            rightBackDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftFrontDrive.setPower(Math.abs(speed));
            leftBackDrive.setPower(Math.abs(speed));
            rightFrontDrive.setPower(Math.abs(speed));
            rightBackDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftFrontDrive.isBusy() && rightFrontDrive.isBusy() && leftBackDrive.isBusy() && rightBackDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        leftFrontDrive.getCurrentPosition(),
                        rightFrontDrive.getCurrentPosition(),
                        leftBackDrive.getCurrentPosition(),
                        rightBackDrive.getCurrentPosition());

                telemetry.update();
            }

            // Stop all motion;
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

}