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
    DcMotor fastSlideMotor = null;
    DcMotor winchMotor = null;
    DcMotor slowSlideMotor = null;
    Servo linearActuator = null;
    Servo leftClaw = null;
    Servo rightClaw = null;

    //change later according to how things go

    public Robot1_Hardware(HardwareMap hardwareMap) {
        //constructs hardware objects based on configuration
        leftDrive = hardwareMap.get(DcMotor.class, "ld");
        rightDrive = hardwareMap.get(DcMotor.class, "rd");
        middleDrive = hardwareMap.get(DcMotor.class, "md");
        fastSlideMotor = hardwareMap.get(DcMotor.class, "fsm");
        winchMotor = hardwareMap.get(DcMotor.class, "wm");
        linearActuator = hardwareMap.get(Servo.class, "la");
        leftClaw = hardwareMap.get(Servo.class, "lc");
        rightClaw = hardwareMap.get(Servo.class, "rc");
        slowSlideMotor = hardwareMap.get(DcMotor.class, "ssm");

    }

    public void initHardware() {
        // called during init() of opMode
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        middleDrive.setDirection(DcMotor.Direction.FORWARD);
        fastSlideMotor.setDirection(DcMotor.Direction.FORWARD);
        slowSlideMotor.setDirection(DcMotor.Direction.FORWARD);
        winchMotor.setDirection(DcMotor.Direction.FORWARD);
        rightClaw.setPosition(0);
        leftClaw.setPosition(0);
    }

}
