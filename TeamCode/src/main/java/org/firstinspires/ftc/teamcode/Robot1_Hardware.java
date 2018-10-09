package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot1_Hardware {
    // Declaring the motors
    DcMotor leftDrive = null;
    DcMotor rightDrive = null;
    DcMotor middleDrive = null;


    public Robot1_Hardware(HardwareMap hardwareMap) {
        //constructs hardware objects based on configuration
        leftDrive = hardwareMap.get(DcMotor.class, "ld");
        rightDrive = hardwareMap.get(DcMotor.class, "rd");
        middleDrive = hardwareMap.get(DcMotor.class, "md");

    }

    public void initHardware() {
        // called during init() of opMode
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        middleDrive.setDirection(DcMotor.Direction.REVERSE);
    }

}