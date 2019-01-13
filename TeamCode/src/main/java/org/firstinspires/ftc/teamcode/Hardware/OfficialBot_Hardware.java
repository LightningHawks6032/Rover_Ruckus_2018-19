package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Vision.Detectors.GoldAlignDetector;
import org.firstinspires.ftc.teamcode.Vision.Detectors.NavTargetDetector;

public class OfficialBot_Hardware implements RobotHardware {
    // Declaring the hardware components
    public OmniSlideDrive drivetrain;
    public OfficialBot_Intake intake;
    public OfficialBot_Outtake outtake;
    public GoldAlignDetector mineralDetector;
    public NavTargetDetector navTargetDetector;
    public ModernRoboticsI2cRangeSensor rangeSensor;
    public Servo markerArm;

    // Servo constants
    public final double MARKER_ARM_UP = 1,
                        MARKER_ARM_MIDDLE = 0.5,
                        MARKER_ARM_DOWN = 0;

    public final double PHONE_SERVO_UP = 1;


    public final double WHEEL_DIAMETER = 4.0;

    // Constants for phone position for nav targets (TUNE)
    public final static double CAMERA_FORWARD_POSITION = 3.5, // eg: Camera is 0 inches in front of robot center
            CAMERA_LEFT_POSITION = 0, // eg: Camera is 0 inches left of the robot's center line
            CAMERA_VERTICAL_POSITION = 0; // eg: Camera is 0 inches from the ground

    // X-position pixel value for center of robot (for mineral sampling)
    public final static int ROBOT_CENTER_X = 285; // TUNE

    public OfficialBot_Hardware(HardwareMap hardwareMap, Gamepad driveGamepad, Gamepad manipsGamepad, boolean gyro){
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

        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rs");

        mineralDetector = new GoldAlignDetector(ROBOT_CENTER_X, 300, 300, true);
        navTargetDetector = new NavTargetDetector(hardwareMap, CAMERA_FORWARD_POSITION, CAMERA_LEFT_POSITION, CAMERA_VERTICAL_POSITION);


        intake = new OfficialBot_Intake(hardwareMap.get(DcMotor.class, "harv"),
                                   hardwareMap.get(DcMotor.class, "flip"),
                                   hardwareMap.get(DcMotor.class, "hs"),
                                   manipsGamepad
        );
        outtake = new OfficialBot_Outtake(hardwareMap.get(DcMotor.class, "lv"),
                                     hardwareMap.get(DcMotor.class, "rv"),
                                     hardwareMap.get(Servo.class, "dump"),
                                     hardwareMap.get(Servo.class, "lp"),
                                     hardwareMap.get(Servo.class, "rp"),
                                     manipsGamepad
        );

        markerArm = hardwareMap.get(Servo.class, "ma");

    }

    public void initHardware() {
        // called during init() of opMode
        drivetrain.initHardware();
        intake.initHardware();
        outtake.initHardware();
        markerArm.setPosition(MARKER_ARM_UP);
    }
}
