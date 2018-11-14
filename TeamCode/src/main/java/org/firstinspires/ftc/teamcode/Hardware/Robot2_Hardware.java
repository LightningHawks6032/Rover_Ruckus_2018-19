package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.HardwareInterface;

public class Robot2_Hardware implements HardwareInterface {
    // motors/servos (omniwheel drivetrain?)
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;
    public DcMotor middleDrive = null;
    public DcMotor winchMotor = null;

    public Robot2_Hardware(HardwareMap hardwareMap){
        leftDrive = hardwareMap.get(DcMotor.class, "ld");
        rightDrive = hardwareMap.get(DcMotor.class, "rd");
        middleDrive = hardwareMap.get(DcMotor.class, "md");

        winchMotor = hardwareMap.get(DcMotor.class, "wm");
    }

    public void initHardware(){
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        middleDrive.setDirection(DcMotor.Direction.REVERSE);

        winchMotor.setDirection(DcMotor.Direction.FORWARD);
    }
}
