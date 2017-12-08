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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 * <p>
 * The code REQUIRES that you DO have encoders on the wheels,
 * otherwise you would use: PushbotAutoDriveByTime;
 * <p>
 * This code ALSO requires that the drive Motors have been configured such that a positive
 * power command moves them forwards, and causes the encoders to count UP.
 * <p>
 * The desired path in this example is:
 * - Drive forward for 48 inches
 * - Spin right for 12 Inches
 * - Drive Backwards for 24 inches
 * - Stop and close the claw.
 * <p>
 * The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 * that performs the actual movement.
 * This methods assumes that each movement is relative to the last stopping place.
 * There are other ways to perform encoder based moves, but this method is probably the simplest.
 * This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name = "Auto Drive By Encoder", group = "Pushbot")
@Disabled
public class EncoderTest extends LinearOpMode {

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

    private static final double PULSES_PER_MOTOR_REV = 7;
    private static final double MOTOR_GEAR = 20;
    private static final double COUNTS_PER_MOTOR_REV = 140;
    private static final double WHEEL_DIAMETER_INCHES = 3;     // For figuring circumference
    private static final double COUNTS_PER_INCH = COUNTS_PER_MOTOR_REV / (WHEEL_DIAMETER_INCHES * 3.1415);
    private static final double SPEED = 0.4;
    private static final double DISTANCE1 = 72;
    private static final long STRAFE_TIME1 = 500;
    private static final long STRAFE_TIME2 = 500;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
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

        jewelArm.setPosition(1);

        leftClaw.setPosition(0); //closes the claw with lb
        rightClaw.setPosition(0.5);

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        //reverses direction of right motors, change if the direction is wrong

        // Send telemetry message to signify robot waiting;
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

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        if (opModeIsActive()) {
            // Step through each leg of the path,
            // Note: Reverse movement is obtained by setting a negative distance (not speed)
            encoderDrive(DISTANCE1, 5.0);  // S1: Forward 12 Inches with 5 Sec timeout
            strafeBackDrive.setPower(SPEED);
            strafeFrontDrive.setPower(SPEED);
            sleep(STRAFE_TIME2);
            strafeBackDrive.setPower(0);
            strafeFrontDrive.setPower(0);
            telemetry.addData("Path", "Complete");
            telemetry.update();
        }
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
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
