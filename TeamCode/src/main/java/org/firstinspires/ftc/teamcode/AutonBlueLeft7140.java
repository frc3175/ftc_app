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
 * Created by tonyp on 10/28/2017.
 */
@Autonomous(name="AutonBlueLeft7140", group="Autonomous")
public class AutonBlueLeft7140 extends LinearOpMode{
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
    private static final double MOTOR_POWER = 0.5;
    private static final double STRAFE_POWER = 0.5;
    private static final long TIME = 4000;
    private static final long STRAFE_TIME = 500;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F,0F,0F};
        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;
        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        // bPrevState and bCurrState represent the previous and current state of the button.
        boolean bPrevState = false;
        boolean bCurrState = false;
        // bLedOn represents the state of the LED.
        boolean bLedOn = true;
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
        telemetry.addData("Status", "Initialized");
        // Set the LED in the beginning
        CSensor.enableLed(bLedOn);
        waitForStart();
        runtime.reset();

        while (opModeIsActive()){
            leftBackDrive.setPower(MOTOR_POWER);
            leftFrontDrive.setPower(MOTOR_POWER);
            rightBackDrive.setPower(MOTOR_POWER);
            rightFrontDrive.setPower(MOTOR_POWER);
            sleep(TIME);
            leftBackDrive.setPower(0);
            leftFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            if (CSensor.blue() > CSensor.red() && CSensor.blue() > CSensor.green()) {
                strafeBackDrive.setPower(-STRAFE_POWER);
                strafeFrontDrive.setPower(-STRAFE_POWER);
                sleep(STRAFE_TIME);
                strafeBackDrive.setPower(0);
                strafeFrontDrive.setPower(0);
            }

        }
    }
}