package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by tonyp on 10/28/2017.
 */
@TeleOp(name = "Teleop10545")
public class Teleop10545 extends OpMode{
    public HardwareMap10545 robot = new HardwareMap10545();
    public HardwareMap map = null;
    private ElapsedTime runTime = new ElapsedTime();

    public void init(){
        telemetry.addData("Status", "Initializing");
        robot.init(map);
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
        robot.frontLeftDrive.setPower(leftPower);
        robot.backLeftDrive.setPower(leftPower);
        robot.backRightDrive.setPower(rightPower);
        robot.frontRightDrive.setPower(rightPower);


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
