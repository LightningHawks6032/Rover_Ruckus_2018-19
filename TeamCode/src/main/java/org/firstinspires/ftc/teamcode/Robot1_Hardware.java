package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot1_Hardware {
    // Declaring the motors
    DcMotor leftDrive = null;
    DcMotor rightDrive = null;
    DcMotor middleDrive = null;
    DcMotor slideMotor = null;
    Servo linearActuator = null;
    ////
    //


    public Robot1_Hardware(HardwareMap hardwareMap) {
        //constructs hardware objects based on configuration
        leftDrive = hardwareMap.get(DcMotor.class, "ld");
        rightDrive = hardwareMap.get(DcMotor.class, "rd");
        middleDrive = hardwareMap.get(DcMotor.class, "md");
        slideMotor = hardwareMap.get(DcMotor.class, "sm");
        linearActuator = hardwareMap.get(Servo.class, "la");
    }

    public void initHardware() {
        // called during init() of opMode
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        middleDrive.setDirection(DcMotor.Direction.REVERSE);
        slideMotor.setDirection(DcMotor.Direction.FORWARD);
    }

}
