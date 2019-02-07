package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Vision.Detectors.GoldAlignDetector;
import org.firstinspires.ftc.teamcode.Vision.Detectors.NavTargetDetector;

public class QualBot_Hardware implements RobotHardware {
    // Declaring the hardware components
    public OmniSlideDrive drivetrain;
    public QualBot_Intake intake;
    public QualBot_Outtake outtake;
    public GoldAlignDetector mineralDetector;
    public NavTargetDetector navTargetDetector;
    public ModernRoboticsI2cRangeSensor rangeSensor;
    public Servo markerArm;

    // Servo constants
    public final double MARKER_ARM_UP = 1,
                        MARKER_ARM_MIDDLE = 0.5,
                        MARKER_ARM_DOWN = 0;

    // Drivetrain Wheel Diameter
    private final double WHEEL_DIAMETER = 4.0;

    // Constants for phone position for nav targets
    private final double CAMERA_FORWARD_POSITION = 7.5, // eg: Camera is 0 inches in front of robot center
            CAMERA_LEFT_POSITION = 0; // eg: Camera is 0 inches left of the robot's center line

    // X-position pixel value for center of robot (for mineral sampling)
    private final int ROBOT_CENTER_X = 225;

    // Range Sensor Distance from Robot Center (inches)
    public final int RANGE_SENSOR_DISPLACEMENT = 5;

    public QualBot_Hardware(HardwareMap hardwareMap, Gamepad driveGamepad, Gamepad manipsGamepad, boolean calibrateSensors) {
        //constructs hardware objects based on configuration
        drivetrain = new OmniSlideDrive(
                hardwareMap.get(DcMotor.class, "ld"), // left drive motor
                hardwareMap.get(DcMotor.class, "rd"), // right drive motor
                hardwareMap.get(DcMotor.class, "md"), // middle drive motor
                new MRGyro(hardwareMap.get(GyroSensor.class, "gs"), calibrateSensors),
                new ExpansionHubIMU(hardwareMap.get(BNO055IMU.class, "imu"), calibrateSensors),
                driveGamepad,
                WHEEL_DIAMETER
        );

        intake = new QualBot_Intake(hardwareMap.get(DcMotor.class, "harv"),
                                   hardwareMap.get(DcMotor.class, "flip"),
                                   hardwareMap.get(DcMotor.class, "hs"),
                                   manipsGamepad
        );
        outtake = new QualBot_Outtake(hardwareMap.get(DcMotor.class, "lv"),
                                     hardwareMap.get(DcMotor.class, "rv"),
                                     hardwareMap.get(Servo.class, "lsv"),
                                     hardwareMap.get(Servo.class, "rsv"),
                                     manipsGamepad
        );

        markerArm = hardwareMap.get(Servo.class, "ma");

        mineralDetector = new GoldAlignDetector(ROBOT_CENTER_X, 325, 300, true);
        navTargetDetector = new NavTargetDetector(hardwareMap, CAMERA_FORWARD_POSITION, CAMERA_LEFT_POSITION);
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rs");
        rangeSensor.setI2cAddress(I2cAddr.create8bit(0x1c));
    }

    public void initHardware() {
        // called during init() of opMode
        drivetrain.initHardware();
        intake.initHardware();
        outtake.initHardware();
        markerArm.setPosition(MARKER_ARM_UP);
    }
}
