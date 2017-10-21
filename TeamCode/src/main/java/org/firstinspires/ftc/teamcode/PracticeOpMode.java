package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import static android.R.attr.left;
import static android.R.attr.right;

/**
 * Created by tonyp on 10/21/2017.
 */
@TeleOp (name = "PracticeOpMode")
public class PracticeOpMode extends OpMode{
    private DcMotor leftDrive = null; //use left stick to go forward/back, use right stick to turn
    private DcMotor rightDrive = null;
    private DcMotor strafeDrive = null; //use bars
    private static final double TURN_POWER = 0.5
    @Override
    public void init() {
        leftDrive = hardwareMap.get(DcMotor.class, "LeftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "RightDrive");
        strafeDrive = hardwareMap.get(DcMotor.class, "StrafeDrive");

        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        //reverses direction of right motors, change if the direction is wrong
    }


    @Override
    public void loop() {
        double leftTurn = gamepad1.right_stick_x;
        leftDrive.setPower(-gamepad1.left_stick_y); //sets left stick to fwd drive
        rightDrive.setPower(-gamepad1.left_stick_y);
        if (gamepad1.left_bumper) {
            strafeDrive.setPower(0.5); //makes left bumper set strafe motors to positive

            if (gamepad1.right_bumper) {
                strafeDrive.setPower(-0.5); //makes right bumper set strafe motors to negative
        }


        }
    }
}
