package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Vision.Detectors.GoldAlignDetector;
import org.firstinspires.ftc.teamcode.Vision.Detectors.NavTargetDetector;

public class StatesBot_Hardware {
    // Declaring the hardware components
    public MecanumWheelDrive drivetrain;
    public StatesBot_Intake intake;
    public StatesBot_Outtake outtake;
    public GoldAlignDetector mineralDetector;
    public NavTargetDetector navTargetDetector;
    public ModernRoboticsI2cRangeSensor rangeSensor;
    //public Servo markerArm;

    // Constants for phone position for nav targets *THESE WILL NEED TO CHANGE AS THEY ARE CURRENTLY PULLED FROM QUALBOT*
    private final double CAMERA_FORWARD_POSITION = 3.5, // eg: Camera is 0 inches in front of robot center
            CAMERA_LEFT_POSITION = 0; // eg: Camera is 0 inches left of the robot's center line

    // X-position pixel value for center of robot (for mineral sampling)
    private final int ROBOT_CENTER_X = 225;

    // Range Sensor Distance from Robot Center (inches)
    public final int RANGE_SENSOR_DISPLACEMENT = 7;


    public StatesBot_Hardware(HardwareMap hardwareMap, Gamepad driveGamepad, Gamepad manipsGamepad, boolean calibrateSensors) {
        drivetrain = new MecanumWheelDrive(
                hardwareMap.get(DcMotor.class, "lfront"),
                hardwareMap.get(DcMotor.class, "rfront"),
                hardwareMap.get(DcMotor.class, "lback"),
                hardwareMap.get(DcMotor.class, "rback"),
                new ExpansionHubIMU(hardwareMap.get(BNO055IMU.class, "imu"), calibrateSensors),
                new MRGyro(hardwareMap.get(GyroSensor.class, "gs"), calibrateSensors),
                driveGamepad
        );

        intake = new StatesBot_Intake(hardwareMap.get(DcMotor.class, "harv"),
                hardwareMap.get(Servo.class, "lflip"),
                hardwareMap.get(Servo.class, "rflip"),
                hardwareMap.get(DcMotor.class, "hs"),
                manipsGamepad
        );
        outtake = new StatesBot_Outtake(hardwareMap.get(DcMotor.class, "lv"),
                hardwareMap.get(DcMotor.class, "rv"),
                hardwareMap.get(Servo.class, "lsv"),
                hardwareMap.get(Servo.class, "rsv"),
                manipsGamepad
        );

        mineralDetector = new GoldAlignDetector(ROBOT_CENTER_X, 160, 300, true, false);
        navTargetDetector = new NavTargetDetector(hardwareMap, CAMERA_FORWARD_POSITION, CAMERA_LEFT_POSITION);
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rs");
        rangeSensor.setI2cAddress(I2cAddr.create8bit(0x1c));
    }

    public void initHardware() {
        // called during init() of opMode
        drivetrain.initHardware();
        intake.initHardware();
        outtake.initHardware();
        //markerArm.setPosition(MARKER_ARM_UP);
    }
}
