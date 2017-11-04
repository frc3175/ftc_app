package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by tonyp on 10/28/2017.
 */
@Autonomous(name="AutonBlueRight7140", group="Autonomous")
public class AutonRedLeft7140 extends LinearOpMode{
    public HardwareMap7140 robot = new HardwareMap7140();
    public HardwareMap map = null;
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
        robot.init(map);
        telemetry.addData("Status", "Initialized");
        // Set the LED in the beginning
        robot.CSensor.enableLed(bLedOn);
        waitForStart();
        runtime.reset();

        while (opModeIsActive()){
            robot.leftBackDrive.setPower(MOTOR_POWER);
            robot.leftFrontDrive.setPower(MOTOR_POWER);
            robot.rightBackDrive.setPower(MOTOR_POWER);
            robot.rightFrontDrive.setPower(MOTOR_POWER);
            sleep(TIME);
            robot.leftBackDrive.setPower(0);
            robot.leftFrontDrive.setPower(0);
            robot.rightBackDrive.setPower(0);
            robot.rightFrontDrive.setPower(0);
            if (robot.CSensor.red() > robot.CSensor.blue() && robot.CSensor.red() > robot.CSensor.green()) {
                robot.strafeBackDrive.setPower(-STRAFE_POWER);
                robot.strafeFrontDrive.setPower(-STRAFE_POWER);
                sleep(STRAFE_TIME);
                robot.strafeBackDrive.setPower(0);
                robot.strafeFrontDrive.setPower(0);
            }

        }
    }
}