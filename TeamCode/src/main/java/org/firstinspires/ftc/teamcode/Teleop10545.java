package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by tonyp on 10/28/2017.
 */
@TeleOp(name = "Teleop10545", group="TeleOp")
public class Teleop10545 extends OpMode{
    private ElapsedTime runTime = new ElapsedTime();

    private DcMotor frontRightDrive = null; // declares the variable frontRightDrive
    private DcMotor frontLeftDrive = null; // declares the variable frontLeftDrive
    private DcMotor backLeftDrive = null; // declares the variable backLeftDrive
    private DcMotor backRightDrive = null; //declares the variable backRightDrive

    private DcMotor upDownArm = null;
    private DcMotor inOutArm = null;

    private Servo leftClaw = null;
    private Servo rightClaw = null;
    private static final double clawOpen = 0;
    private static final double clawClosed = 0.5;

    public void init(){
        telemetry.addData("Status", "Initializing");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive"); // initializing frontRightDrive
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive"); // initializing frontLeftDrive
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive"); // initializing backRightDrive
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive"); // initializing bckLeftDrive
        frontRightDrive.setDirection(DcMotorSimple.Direction.REVERSE); // reverses direction of the frontLeftDrive
        backRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        upDownArm = hardwareMap.get(DcMotor.class, "upDownArm");
        inOutArm = hardwareMap.get(DcMotor.class, "inOutArm");

        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        telemetry.addData("Status", "Initialized");
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
        runTime.reset();
    }

    @Override
    public void loop() {
        double leftPower;
        double rightPower;
        leftPower = -gamepad1.left_stick_y;
        rightPower = -gamepad1.right_stick_y;
        frontLeftDrive.setPower(leftPower);
        backLeftDrive.setPower(leftPower);
        backRightDrive.setPower(rightPower);
        frontRightDrive.setPower(rightPower);

        upDownArm.setPower(-gamepad2.left_stick_y);
        inOutArm.setPower(gamepad2.right_stick_x);
        if (gamepad2.b) {
            rightClaw.setPosition(clawOpen);
            leftClaw.setPosition(clawClosed);
        } else if(gamepad2.x) {
            rightClaw.setPosition(clawClosed);
            leftClaw.setPosition(clawOpen);
        }

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runTime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);

    }

    /*
     * Code to run ONCE after the driver hits STOP
    */
    @Override
    public void stop() {
    }
}
