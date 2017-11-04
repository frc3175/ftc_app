package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by tonyp on 10/28/2017.
 */

public class HardwareMap7140 {

    public DcMotor leftFrontDrive = null; //use left stick to go forward/back, use right stick to turn
    public DcMotor leftBackDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor rightBackDrive = null;
    public DcMotor strafeFrontDrive = null; //use bars
    public DcMotor strafeBackDrive = null;
    public DcMotor arm = null;
    public Servo leftClaw = null;
    public Servo rightClaw = null;
    public ColorSensor CSensor = null;
    public HardwareMap HMap = null; //name HardwareMap to HMap

    public HardwareMap7140(){

    }

    public void init(HardwareMap map){
        this.HMap = map;
        leftFrontDrive = HMap.get(DcMotor.class, "LeftFrontDrive"); //initializes motors
        leftBackDrive = HMap.get(DcMotor.class, "LeftBackDrive");
        rightFrontDrive = HMap.get(DcMotor.class, "RightFrontDrive");
        rightBackDrive = HMap.get(DcMotor.class, "RightBackDrive");
        strafeFrontDrive = HMap.get(DcMotor.class, "StrafeFrontDrive");
        strafeBackDrive = HMap.get(DcMotor.class, "StrafeBackDrive");
        arm = HMap.get(DcMotor.class, "Arm");
        leftClaw = HMap.get(Servo.class, "LeftClaw");
        rightClaw = HMap.get(Servo.class, "RightClaw");

        CSensor = HMap.get(ColorSensor.class, "ColorSensor");

        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        //reverses direction of right motors, change if the direction is wrong
    }
}
