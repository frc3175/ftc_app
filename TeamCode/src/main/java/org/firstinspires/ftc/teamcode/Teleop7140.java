package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by tonyp on 10/21/2017.
 */
@TeleOp (name = "Teleop7140", group="TeleOp")
public class Teleop7140 extends OpMode{

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftFrontDrive = null; //use left stick to go forward/back, use right stick to turn
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor strafeBackDrive = null;
    private DcMotor strafeFrontDrive = null;
    private DcMotor arm = null;
    private Servo leftClaw = null;
    private Servo rightClaw = null;
    private Servo jewelArm = null;
    private ColorSensor colorSensor = null;
    private static final double TURN_POWER = 0.75;//sets constant for turn power
    private static final double ARM_POWER = 0.5; //sets arm power constant

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");//tells the user(s) that the robot is initializing

        leftFrontDrive = hardwareMap.get(DcMotor.class, "LeftFrontDrive"); //initializes motors
        leftBackDrive = hardwareMap.get(DcMotor.class, "LeftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "RightBackDrive");
        strafeBackDrive = hardwareMap.get(DcMotor.class, "StrafeBackDrive");
        strafeFrontDrive = hardwareMap.get(DcMotor.class, "StrafeFrontDrive");
        arm = hardwareMap.get(DcMotor.class, "Arm");
        leftClaw = hardwareMap.get(Servo.class, "LeftClaw");
        rightClaw = hardwareMap.get(Servo.class, "RightClaw");
        jewelArm = hardwareMap.get(Servo.class, "JewelArm");

        colorSensor = hardwareMap.get(ColorSensor.class, "ColorSensor");
        colorSensor.enableLed(false);

        jewelArm.setPosition(1);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        //reverses direction of right motors, change if the direction is wrong
        leftClaw.setPosition(0.5);
        rightClaw.setPosition(0);

        telemetry.addData("Status", "Initialized"); //tells the user(s) init. is finished
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        double leftPower = -gamepad1.left_stick_y;//defines variables used to move up&down
        double rightPower = -gamepad1.left_stick_y;
        leftPower += gamepad1.left_stick_x * TURN_POWER; //enables turns
        rightPower -= gamepad1.left_stick_x * TURN_POWER;//uses right stick left&right
        leftFrontDrive.setPower(leftPower);//enables forward&backward movement
        leftBackDrive.setPower(leftPower);//uses left stick up&down
        rightFrontDrive.setPower(rightPower);
        rightBackDrive.setPower(rightPower);
        strafeFrontDrive.setPower(gamepad1.right_stick_x); //enables strafing
        strafeBackDrive.setPower(gamepad1.right_stick_x);

        if (gamepad2.b){
            arm.setPower(ARM_POWER); //makes the b button raise the arm
        } else if (gamepad2.a){
            arm.setPower(-ARM_POWER); //makes the a button lower the arm
        } else {
            arm.setPower(0); //makes sure that the motor is not moving if a or b is not pressed
        }

        if (gamepad2.left_bumper) {
            leftClaw.setPosition(0); //closes the claw with lb
            rightClaw.setPosition(0.5);
        } else if (gamepad2.right_bumper){
            leftClaw.setPosition(0.45); //opens the claw with rb
            rightClaw.setPosition(0.05);
        }
        if (gamepad2.a) {
            jewelArm.setPosition(0.65);
        } else if (gamepad2.y) {
            jewelArm.setPosition(1);
        }
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}