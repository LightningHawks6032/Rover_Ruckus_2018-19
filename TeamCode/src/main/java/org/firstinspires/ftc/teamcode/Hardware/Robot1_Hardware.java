package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.sun.tools.javac.util.ForwardingDiagnosticFormatter;

import org.firstinspires.ftc.teamcode.Hardware.HardwareInterface;
import org.firstinspires.ftc.teamcode.Vision.Detectors.GoldAlignDetector;
import org.firstinspires.ftc.teamcode.Vision.Detectors.NavTargetDetector;

public class Robot1_Hardware implements HardwareInterface {
    // Declaring the motors
    public OmniSlideDrive drivetrain;
    public GoldAlignDetector mineralDetector;
    public NavTargetDetector navTargetDetector;
    public DcMotor slideMotor = null;
    public DcMotor hangVex = null;
    public DcMotor hangNvst = null;
    public Servo linearActuator = null;
    public Servo leftClaw = null;
    public Servo rightClaw = null;
    public Servo markerArm = null;
    public Encoder slideEncoder = null;


    // Servo constants
    public final double RIGHT_CLAW_CLOSE = 1,
           RIGHT_CLAW_OPEN = 0,
           LEFT_CLAW_CLOSE = 0,
           LEFT_CLAW_OPEN = 1,
           MARKER_ARM_UP = 1,
           MARKER_ARM_MIDDLE = 0.5,
           MARKER_ARM_DOWN = 0;

    public double wheelDiameter = 4.0;

    // Constants for phone position for nav targets
    public final static int CAMERA_FORWARD_POSITION = 0, // eg: Camera is 0 mm in front of robot center
                 CAMERA_VERTICAL_POSITION = 0, // eg: Camera is 0 mm above ground
                 CAMERA_LEFT_POSITION = 0; // eg: Camera is 0 mm left of the robot's center line

    // X-position pixel value for center of robot
    public final static int ROBOT_CENTER_X = 230;

    /**
     * Constructs each hardware object
     * @param hardwareMap : hardwareMap given in any LinearOpMode or OpMode
     * @param driveGamepad : gamepad (either gamepad1 or gamepad2 used for the drivetrain)
     * @param gyro : whether or not we want to calibrate the gyro (true in autonomous, typically false in tele-op modes)
     */
    public Robot1_Hardware(HardwareMap hardwareMap, Gamepad driveGamepad, boolean gyro) {
        //constructs hardware objects based on configuration
        if (gyro) {
            drivetrain = new OmniSlideDrive(
                    hardwareMap.get(DcMotor.class, "ld"), // left drive motor
                    hardwareMap.get(DcMotor.class, "rd"), // right drive motor
                    hardwareMap.get(DcMotor.class, "md"), // middle drive motor
                    new MRGyro(hardwareMap.get(GyroSensor.class, "gs"), true), // gyro sensor calibrated
                    driveGamepad,
                    wheelDiameter
            );
        } else {
            drivetrain = new OmniSlideDrive(
                    hardwareMap.get(DcMotor.class, "ld"), // left drive motor
                    hardwareMap.get(DcMotor.class, "rd"), // right drive motor
                    hardwareMap.get(DcMotor.class, "md"), // middle drive motor
                    new MRGyro(hardwareMap.get(GyroSensor.class, "gs"), false), // gyro sensor not calibrated
                    driveGamepad,
                    wheelDiameter
            );
        }

        mineralDetector = new GoldAlignDetector(ROBOT_CENTER_X, 100, true);
        navTargetDetector = new NavTargetDetector(hardwareMap, CAMERA_FORWARD_POSITION, CAMERA_VERTICAL_POSITION, CAMERA_LEFT_POSITION, true);

        slideMotor = hardwareMap.get(DcMotor.class, "sm");
        slideEncoder = new Encoder(slideMotor, "neverest", 0);


        linearActuator = hardwareMap.get(Servo.class, "la");
        leftClaw = hardwareMap.get(Servo.class, "lc");
        rightClaw = hardwareMap.get(Servo.class, "rc");
        markerArm = hardwareMap.get(Servo.class, "ma");

        hangVex = hardwareMap.get(DcMotor.class, "hv");
        hangNvst = hardwareMap.get(DcMotor.class, "hn");
    }

    public void initHardware() {
        // called during init() of opMode
        drivetrain.setupMotors();
        slideMotor.setDirection(DcMotor.Direction.FORWARD);
        hangVex.setDirection(DcMotor.Direction.FORWARD);
        hangNvst.setDirection(DcMotor.Direction.FORWARD);
        rightClaw.setPosition(RIGHT_CLAW_CLOSE);
        leftClaw.setPosition(LEFT_CLAW_CLOSE);
        markerArm.setPosition(MARKER_ARM_UP);
    }

}
