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
    public DcMotor hangUMotor = null;
    public DcMotor hangDMotor = null;
    public Servo clawServo = null;

    // variable declaration
    public final double CLAW_CLOSE = 0,
                        CLAW_OPEN = 1;

    public Robot2_Hardware(HardwareMap hardwareMap){
        leftDrive = hardwareMap.get(DcMotor.class, "ld");
        rightDrive = hardwareMap.get(DcMotor.class, "rd");
        middleDrive = hardwareMap.get(DcMotor.class, "md");

        winchMotor = hardwareMap.get(DcMotor.class, "wm");
        hangUMotor = hardwareMap.get(DcMotor.class, "um"); // may be a vex motor
        hangDMotor = hardwareMap.get(DcMotor.class, "dm"); // possibly this one

        clawServo = hardwareMap.get(Servo.class, "cs");
    }

    public void initHardware(){
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        middleDrive.setDirection(DcMotor.Direction.REVERSE);

        winchMotor.setDirection(DcMotor.Direction.FORWARD);
        hangUMotor.setDirection(DcMotor.Direction.FORWARD);
        hangDMotor.setDirection(DcMotor.Direction.FORWARD);

        clawServo.setPosition(CLAW_OPEN);
    }
}
