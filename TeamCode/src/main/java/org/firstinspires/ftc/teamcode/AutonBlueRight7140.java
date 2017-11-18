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
@Autonomous(name="AutonBlueRight7140", group="Autonomous")
@Disabled
public class AutonBlueRight7140 extends LinearOpMode{

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
    private static final double SPEED = 0.5;
    private static final long STRAFE_TIME1 = 500;
    private static final long STRAFE_TIME2 = 2000;
    private static final double CPR = 1440; // encoder counts per motor revolution
    private static final double DISTANCE1 = 12; //distance between the balancing stone and the colored balls
    private static final double DIAMETER = 4.0; // For figuring circumference
    private static final double CIRCUMFERENCE = DIAMETER * 3.141592653535;
    private static final double COUNTS_PER_INCH = CPR / CIRCUMFERENCE;

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

        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        //reverses direction of right motors, change if the direction is wrong

        // Set the LED in the beginning
        CSensor.enableLed(true);

        leftClaw.setPosition(0.5);
        rightClaw.setPosition(0);

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

        while (opModeIsActive()){
            encoderDrive(DISTANCE1);

            if (CSensor.blue() > CSensor.red() && CSensor.blue() > CSensor.green()) {
                strafeBackDrive.setPower(-SPEED);
                strafeFrontDrive.setPower(-SPEED);
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
    }

    private void encoderDrive(double distance) throws InterruptedException {
        int leftPosition = (int) (leftFrontDrive.getCurrentPosition() + COUNTS_PER_INCH * distance);
        int rightPosition = (int) (rightFrontDrive.getCurrentPosition() + COUNTS_PER_INCH * distance);

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            leftFrontDrive.setTargetPosition(leftPosition);
            leftBackDrive.setTargetPosition(leftPosition);
            rightFrontDrive.setTargetPosition(rightPosition);
            rightBackDrive.setTargetPosition(rightPosition);
            // Turn On RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFrontDrive.setPower(SPEED);
            leftBackDrive.setPower(SPEED);
            rightFrontDrive.setPower(SPEED);
            rightBackDrive.setPower(SPEED);
            //sleep(50);
            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
        }
    }

}