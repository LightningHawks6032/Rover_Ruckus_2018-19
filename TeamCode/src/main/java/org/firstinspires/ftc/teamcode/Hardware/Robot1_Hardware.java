package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.sun.tools.javac.util.ForwardingDiagnosticFormatter;

import org.firstinspires.ftc.teamcode.Hardware.HardwareInterface;

public class Robot1_Hardware implements HardwareInterface {
    // Declaring the motors
    public OmniSlideDrive drivetrain;
    public DcMotor fastSlideMotor = null;
    public DcMotor winchMotor = null;
    public DcMotor slowSlideMotor = null;
    public Servo linearActuator = null;
    public Servo leftClaw = null;
    public Servo rightClaw = null;
    public Servo markerArm = null;
    public Encoder slowSlideEncoder = null;
    private Encoder fastSlideEncoder = null;

    // Servo constants
    public final double RIGHT_CLAW_CLOSE = 1,
           RIGHT_CLAW_OPEN = 0,
           LEFT_CLAW_CLOSE = 0,
           LEFT_CLAW_OPEN = 1,
           MARKER_ARM_UP = 1,
           MARKER_ARM_DOWN = 0;

    public double wheelDiameter = 4.0;

    // Constants for phone position for nav targets
    public final static int CAMERA_FORWARD_POSITION = 0, // eg: Camera is 0 mm in front of robot center
                 CAMERA_VERTICAL_POSITION = 0, // eg: Camera is 0 mm above ground
                 CAMERA_LEFT_POSITION = 0; // eg: Camera is 0 mm left of the robot's center line


    // X-position pixel value for center of robot
    public final static int ROBOT_CENTER_X = 230;

    public Robot1_Hardware(HardwareMap hardwareMap, Gamepad driveGamepad) {
        //constructs hardware objects based on configuration
        drivetrain = new OmniSlideDrive(
                hardwareMap.get(DcMotor.class, "ld"), // left drive motor
                hardwareMap.get(DcMotor.class, "rd"), // right drive motor
                hardwareMap.get(DcMotor.class, "md"), // middle drive motor
                new MRGyro(hardwareMap.get(GyroSensor.class, "gs")), // gyro sensor
                driveGamepad,
                wheelDiameter
        );

        fastSlideMotor = hardwareMap.get(DcMotor.class, "fsm");
        winchMotor = hardwareMap.get(DcMotor.class, "wm");
        linearActuator = hardwareMap.get(Servo.class, "la");
        leftClaw = hardwareMap.get(Servo.class, "lc");
        rightClaw = hardwareMap.get(Servo.class, "rc");
        slowSlideMotor = hardwareMap.get(DcMotor.class, "ssm");
        markerArm = hardwareMap.get(Servo.class, "ma");

    }

    public void initHardware() {
        // called during init() of opMode
        drivetrain.setupMotors();
        fastSlideMotor.setDirection(DcMotor.Direction.FORWARD);
        slowSlideMotor.setDirection(DcMotor.Direction.FORWARD);
        winchMotor.setDirection(DcMotor.Direction.FORWARD);
        rightClaw.setPosition(RIGHT_CLAW_CLOSE);
        leftClaw.setPosition(LEFT_CLAW_CLOSE);
        markerArm.setPosition(MARKER_ARM_UP);
    }

}
