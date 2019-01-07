package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;

public class Robot2_Hardware implements RobotHardware {
    // Declaring the hardware components
    public OmniSlideDrive drivetrain;
    public Robot2_Intake intake;
    public Robot2_Outtake outtake;
    public Servo markerArm;
    public Servo phoneServo;

    // Servo constants
    public final double MARKER_ARM_UP = 1,
                        MARKER_ARM_MIDDLE = 0.5,
                        MARKER_ARM_DOWN = 0;

    public final double PHONE_SERVO_UP = 1;

    public final double WHEEL_DIAMETER = 4.0;

    public Robot2_Hardware(HardwareMap hardwareMap, Gamepad driveGamepad, Gamepad manipsGamepad, boolean gyro){
        //constructs hardware objects based on configuration
        if (gyro) {
            drivetrain = new OmniSlideDrive(
                    hardwareMap.get(DcMotor.class, "ld"), // left drive motor
                    hardwareMap.get(DcMotor.class, "rd"), // right drive motor
                    hardwareMap.get(DcMotor.class, "md"), // middle drive motor
                    new MRGyro(hardwareMap.get(GyroSensor.class, "gs"), true), // gyro sensor calibrated
                    driveGamepad,
                    WHEEL_DIAMETER
            );
        } else {
            drivetrain = new OmniSlideDrive(
                    hardwareMap.get(DcMotor.class, "ld"), // left drive motor
                    hardwareMap.get(DcMotor.class, "rd"), // right drive motor
                    hardwareMap.get(DcMotor.class, "md"), // middle drive motor
                    new MRGyro(hardwareMap.get(GyroSensor.class, "gs"), false), // gyro sensor not calibrated
                    driveGamepad,
                    WHEEL_DIAMETER
            );
        }



        intake = new Robot2_Intake(hardwareMap.get(DcMotor.class, "col"),
                                   hardwareMap.get(DcMotor.class, "flip"),
                                   hardwareMap.get(DcMotor.class, "hs"),
                                   manipsGamepad
        );
        outtake = new Robot2_Outtake(hardwareMap.get(DcMotor.class, "lv"),
                                     hardwareMap.get(DcMotor.class, "rv"),
                                     hardwareMap.get(Servo.class, "dump"),
                                     hardwareMap.get(Servo.class, "lp"),
                                     hardwareMap.get(Servo.class, "rp"),
                                     manipsGamepad
        );

        markerArm = hardwareMap.get(Servo.class, "ma");
        phoneServo = hardwareMap.get(Servo.class, "ps");

    }

    public void initHardware() {
        // called during init() of opMode
        drivetrain.initHardware();
        intake.initHardware();
        outtake.initHardware();
        markerArm.setPosition(MARKER_ARM_UP);
        phoneServo.setPosition(PHONE_SERVO_UP);
    }
}
