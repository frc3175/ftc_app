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
@TeleOp (name = "PracticeOpMode", group="TeleOp")
public class PracticeOpMode extends OpMode{

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
    private static final double TURN_POWER = 0.75;//sets constant for turn power
    private static final double STRAFE_POWER = 1;//sets constant for strafe power
    private static final double ARM_POWER = 0.5; //sets arm power constant
    private static final double CLAW_OPEN = .5; //sets constants for claw open/closed positions
    private static final double CLAW_CLOSED = 0;


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

        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        //reverses direction of right motors, change if the direction is wrong

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
        leftPower += gamepad1.right_stick_x * TURN_POWER; //enables turns
        rightPower -= gamepad1.right_stick_x * TURN_POWER;//uses right stick left&right
        leftFrontDrive.setPower(leftPower);//enables forward&backward movement
        leftBackDrive.setPower(leftPower);//uses left stick up&down
        rightFrontDrive.setPower(rightPower);
        rightBackDrive.setPower(rightPower);
        if (gamepad1.left_bumper) {
            strafeFrontDrive.setPower(-STRAFE_POWER);//makes left bumper set strafe motors to positive
            strafeBackDrive.setPower(-STRAFE_POWER);
        } else {
            strafeFrontDrive.setPower(0); //makes not pressing a bumper set strafe motors to 0
            strafeBackDrive.setPower(0);
        }
        if (gamepad1.right_bumper) {
            strafeFrontDrive.setPower(STRAFE_POWER); //makes right bumper set strafe motors to negative
            strafeBackDrive.setPower(STRAFE_POWER);
        } else {
            strafeFrontDrive.setPower(0); //makes not pressing a bumper set strafe motors to 0 (again)
            strafeBackDrive.setPower(0);

        }
        if (gamepad2.b){
            arm.setPower(ARM_POWER); //makes the b button raise the arm
        } else if (gamepad2.a){
            arm.setPower(-ARM_POWER); //makes the a button lower the arm
        } else {
            arm.setPower(0); //makes sure that the motor is not moving if a or b is not pressed
        }
        if (gamepad2.left_bumper) {
            leftClaw.setPosition(CLAW_CLOSED); //closes the claw with lb
            rightClaw.setPosition(CLAW_CLOSED);
        } else if (gamepad2.right_bumper){
            leftClaw.setPosition(CLAW_OPEN); //opens the claw with rb
            rightClaw.setPosition(CLAW_OPEN);
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