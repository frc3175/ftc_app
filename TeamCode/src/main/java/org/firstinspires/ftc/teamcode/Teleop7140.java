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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Teleop7140", group="TeleOp")
public class Teleop7140 extends OpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
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

    private static final double TURN_POWER = 0.75;//sets constant for turn power
    private static final double STRAFE_POWER = 0.5;//sets constant for strafe power
    private static final double ARM_POWER = 0.5; //sets arm power constant
    private static final double CLAW_OPEN = .5; //sets constants for claw open/closed positions
    private static final double CLAW_CLOSED = 0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");
        try {
            leftFrontDrive = hardwareMap.dcMotor.get("LeftFrontDrive");//initializes motors
            telemetry.addData("leftFrontDrive", "success");
            leftBackDrive = hardwareMap.dcMotor.get("LeftBackDrive");
            telemetry.addData("leftBackDrive", "success");
            rightFrontDrive = hardwareMap.dcMotor.get("RightFrontDrive");
            telemetry.addData("rightFrontDrive", "success");
            rightBackDrive = hardwareMap.dcMotor.get("RightBackDrive");
            telemetry.addData("rightBackDrive", "success");
            strafeFrontDrive = hardwareMap.dcMotor.get("StrafeFrontDrive");
            telemetry.addData("strafeFrontDrive", "success");
            strafeBackDrive = hardwareMap.dcMotor.get("StrafeBackDrive");
            telemetry.addData("strafeBackDrive", "success");
            arm = hardwareMap.dcMotor.get("Arm");
            telemetry.addData("arm", "success");
            leftClaw = hardwareMap.servo.get("LeftClaw");
            telemetry.addData("leftClaw", "success");
            rightClaw = hardwareMap.servo.get("RightClaw");
            telemetry.addData("rightClaw", "success");

            CSensor = hardwareMap.colorSensor.get("ColorSensor");
            telemetry.addData("CSensor", "success");
        } catch (ExceptionInInitializerError ex) {
            telemetry.addData("Exception", ex);
        }
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        //reverses direction of right motors, change if the direction is wrong

        // Tell the driver that initialization is complete.
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
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
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
