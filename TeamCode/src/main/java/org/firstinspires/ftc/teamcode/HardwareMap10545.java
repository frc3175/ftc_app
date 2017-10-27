package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by tonyp on 10/27/2017.
 */

/**
 *
 */
public class HardwareMap10545 {
    public DcMotor frontRightDrive = null; // declares the variable frontRightDrive
    public DcMotor frontLeftDrive = null; // declares the variable frontLeftDrive
    public DcMotor backLeftDrive = null; // declares the variable backLeftDrive
    public DcMotor backRightDrive = null; //declares the variable backRightDrive

    public HardwareMap map = null;
    private ElapsedTime period  = new ElapsedTime();

    public HardwareMap10545() { // construction

    }

    public void init(HardwareMap hMap){
        this.map = hMap;
        frontRightDrive = map.get(DcMotor.class, "frontRightDrive"); // initializing frontRightDrive
        frontLeftDrive = map.get(DcMotor.class, "frontLeftDrive"); // initializing frontLeftDrive
        backRightDrive = map.get(DcMotor.class, "backRightDrive"); // initializing backRightDrive
        backLeftDrive = map.get(DcMotor.class, "backLeftDrive"); // initializing bckLeftDrive
        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE); // reverses direction of the frontLeftDrive
    }
}
