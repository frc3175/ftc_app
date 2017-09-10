package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;

/**
 * Created by tonyp on 9/9/2017.
 * This is an example of what the teleop code for this year's robot could look like
 */
@TeleOp(name = "SampleTeleop")
// tells the robot controller what kind of program this is
public class SampleTeleop extends OpMode{
// "extends" gives this program full functions of an OpMode

    private static final double clawPosition1 = 0;
    // constants that you can modify to adjust the position of
    private static final double clawPosition2 = 0.25;   // the servos on the claws

    private DcMotor leftDrive = null;        // declare all the motors, servos, sensors before init
    private DcMotor rightDrive = null;
    private DcMotor frontDrive = null;
    private DcMotor backDrive = null;
    private DcMotor supportArm = null;
    private DcMotor extensionArm = null;
    private Servo leftClaw = null;
    private Servo rightClaw = null;

    private boolean clawOpen = false; // instance variable that stores the state of the claw

    private ElapsedTime runTime = new ElapsedTime();    // how long the method has been running

    /**
     *  This method is called when you click the init button before play. It sets up all the motors,
     *  servos, and sensors to the phone.
     *
     */
    public void init() {

        telemetry.addData("Status", "Initializing");    // display "initializing" on the phones
        telemetry.update();

        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");     // robot components correspond
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");   // to the configured names
        frontDrive = hardwareMap.get(DcMotor.class, "frontDrive");
        backDrive = hardwareMap.get(DcMotor.class, "backDrive");
        supportArm = hardwareMap.get(DcMotor.class, "supportArm");
        extensionArm = hardwareMap.get(DcMotor.class, "extensionArm");
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);  // motors that go to the same direction
        rightDrive.setDirection(DcMotor.Direction.REVERSE); // but face different ways need to be
        frontDrive.setDirection(DcMotor.Direction.FORWARD); // reversed
        backDrive.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized"); // tells the driver it has been initialzed
    }

    @Override
    // resets the timer
    public void start() {
        runTime.reset();
    }

    @Override
     //This method keeps repeating when the driver hits play on the driver station.
    public void loop() {

        leftDrive.setPower(-gamepad1.left_stick_y);     // tank drive
        rightDrive.setPower(-gamepad1.right_stick_y);

        double motorPower;
        motorPower = Math.max(gamepad1.left_stick_x, gamepad1.right_stick_x);
        // it takes the greater value of the two joysticks' x-axises
        frontDrive.setPower(-motorPower);  // the robot slides to the right if the driver moves the
        backDrive.setPower(-motorPower);   // joystick to the right and so on

        supportArm.setPower(gamepad2.right_stick_y);    // controls the rack and pinion slides
        extensionArm.setPower(gamepad2.right_stick_x);

        if (gamepad2.a && clawOpen) {               // closes the claw
            leftClaw.setPosition(clawPosition2);
            rightClaw.setPosition(clawPosition1);
            clawOpen = false;
        } else if (!clawOpen) {                     // opens the claw
            leftClaw.setPosition(clawPosition1);
            rightClaw.setPosition(clawPosition2);
            clawOpen = true;
        }

        telemetry.addData("Status", "Run Time: " + runTime.toString());
        telemetry.addData("voltage", "%.1f volts", new Func<Double>() {
            @Override public Double value() {
                return getBatteryVoltage();
            }
        });
        // displays robot run time and battery voltage on the phones

    }

    @Override
    // executed when the driver stops the robot
    public void stop() {
    }

    /**
     *
     * This method returns the battery voltage of the robot
     */
    private double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }

}
