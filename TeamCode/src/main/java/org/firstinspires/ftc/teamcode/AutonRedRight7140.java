package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by tonyp on 10/28/2017.
 */
@Autonomous(name="AutonBlueRight7140")
public class AutonRedRight7140 extends LinearOpMode{
    public HardwareMap7140 robot = new HardwareMap7140();
    public HardwareMap map = null;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initializing");
        telemetry.update();
        robot.init(map);
        telemetry.addData("Status", "Initialized");
        waitForStart();
        runtime.reset();
        while (opModeIsActive()){
            //all the movement goes here
        }
    }
}