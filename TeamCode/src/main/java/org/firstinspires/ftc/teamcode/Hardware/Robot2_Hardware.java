package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.HardwareInterface;

public class Robot2_Hardware implements HardwareInterface {
    // Declaring the motors
    public OmniSlideDrive drivetrain;
    public Robot2_Intake intake;
    public DcMotor leftVertical;
    public DcMotor rightVertical;
    public Servo leftDumper;
    public Servo rightDumper;

    // Servo constants


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



        intake = new Robot2_Intake(hardwareMap.get(DcMotor.class, "int"),
                                   hardwareMap.get(DcMotor.class, "iflip"),
                                   hardwareMap.get(DcMotor.class, "hs"),
                                   manipsGamepad
        );
        leftVertical = hardwareMap.get(DcMotor.class, "lv");
        rightVertical = hardwareMap.get(DcMotor.class, "rv");

        leftDumper = hardwareMap.get(Servo.class, "ld");
        rightDumper = hardwareMap.get(Servo.class, "rd");


    }

    public void initHardware() {
        // called during init() of opMode
        drivetrain.setupMotors();
        intake.setupMotors();
        leftVertical.setDirection(DcMotor.Direction.FORWARD);
        rightVertical.setDirection(DcMotor.Direction.FORWARD);
    }
}
