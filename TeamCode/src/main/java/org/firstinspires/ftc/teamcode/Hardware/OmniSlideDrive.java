package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.GyroSensor;

import org.firstinspires.ftc.teamcode.FieldMapping.Vector;

public class OmniSlideDrive {
    public Vector robotPos; // Position on field
    public int robotAngle; // Angle relative to (0, 0) on field

    // Motors
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor middleMotor;
    private Encoder leftEncoder;
    private Encoder rightEncoder;
    private Encoder middleEncoder;
    private MRGyro gyroSensor;
    private Gamepad gamepad;

    // Constants to regulate maximum power
    private final double MAX_DRIVE_POWER = 1;
    private final double MAX_MIDDLE_POWER = 1;
    private double boost;

    public OmniSlideDrive(DcMotor lm, DcMotor rm, DcMotor mm, MRGyro gyro, Gamepad gamepad, double wheelDiam) {
        robotPos = new Vector(0, 0); // set default position later in auto
        robotAngle = 0; // set default angle later in auto

        leftMotor = lm;
        rightMotor = rm;
        middleMotor = mm;
        gyroSensor = gyro;
        this.gamepad = gamepad;
        leftEncoder = new Encoder(lm, "Neverest", wheelDiam);
        rightEncoder = new Encoder(rm, "Neverest", wheelDiam);
        middleEncoder = new Encoder(mm, "Neverest", wheelDiam);

        boost = 1;
    }

    public void setRobotPos(double x, double y) {
        robotPos.setX(x);
        robotPos.setY(y);
    }
    public void setRobotAngle(int angle) {
        robotAngle = angle;
    }

    public void setupMotors() {
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        middleMotor.setDirection(DcMotor.Direction.FORWARD);
        encoderSetup();
    }

    private void encoderSetup() {
        leftEncoder.setup();
        rightEncoder.setup();
        middleEncoder.setup();
    }

    // Shortcut method for setting the power of the left drive, right drive, and middle drive
    public void setPowers(double lp, double rp, double mp) {
        leftMotor.setPower(lp);
        rightMotor.setPower(rp);
        middleMotor.setPower(mp);
    }

    public void manageTeleOp() {
        //drive1 controls (slide drive)
        if (gamepad.left_trigger > 0) {
            setPowers(0, 0, -MAX_MIDDLE_POWER); // strafe left
        } else if (gamepad.right_trigger > 0) {
            setPowers(0, 0, MAX_MIDDLE_POWER); // strafe right
        } else {
            // motor power = joysticks
            setPowers(-gamepad.left_stick_y * MAX_DRIVE_POWER * boost, -gamepad.right_stick_y * MAX_DRIVE_POWER * boost, 0);
        }

        applyBoost();
    }

    private void applyBoost() {
        if (gamepad.x)
            boost = 1;
    }


    /**
     * Robot drives forward or backward a set amount of linear distance using encoders
     * @param direction : forward (1) or backward (-1)
     * @param distance : linear distance in inches for the robot to drive over
     * @param pow : constant power at which the robot drives
     */
    public void driveDistance(int direction, double distance, double pow) {
        leftEncoder.reset();
        rightEncoder.reset();

        leftEncoder.runToPosition();
        rightEncoder.runToPosition();

        leftEncoder.setTarget(direction * distance);
        rightEncoder.setTarget(direction * distance);

        setPowers(direction * pow, direction * pow, 0);

        while (leftMotor.isBusy() && rightMotor.isBusy()) {
            // WAIT - Motors are busy
        }

        setPowers(0, 0, 0);
    }

    /**
     * Robot strafes left or right a set amount of linear distance using encoders
     * @param direction : right (1) or left (-1)
     * @param distance : linear distance in inches for the robot to strafe over
     * @param pow : constant power at which the robot strafes
     */
    public void strafeDistance(int direction, double distance, double pow) {
        middleEncoder.reset();

        middleEncoder.runToPosition();

        middleEncoder.setTarget(direction * distance);

        setPowers(0, 0, direction * pow);

        while (middleMotor.isBusy()) {
            // WAIT - Motor is busy
        }

        setPowers(0, 0, 0);
    }

    /**
     * Robot drives/strafes forward for a set amount of time
     * @param pow : constant power at which the robot drives (positive power equals forward/right, negative power equals backward/left)
     * @param seconds : seconds during which the robot drives
     * @throws InterruptedException
     */
    public void driveForTime(double pow, long seconds) throws InterruptedException {
        encoderSetup();
        setPowers(pow, pow, 0);
        Thread.sleep(seconds * 1000);
        setPowers(0, 0, 0);
    }
    public void strafeForTime(double pow, long seconds) throws InterruptedException {
        encoderSetup();
        setPowers(0, 0, pow);
        Thread.sleep(seconds * 1000);
        setPowers(0, 0, 0);
    }

    /**
     * Goes to a specific position on the field by turning first and then driving straight
     * @param location : Vector position of the location, use field map
     * @param pow : power at which we drive
     */
    public void goTo(Vector location, double pow) {
        face(location); // Turn to face location
        driveDistance(1, location.distanceFrom(robotPos), pow); // Drive to location
        robotPos = location; // Update robot position
    }

    /**
     * Robot turns to face certain location on field
     * @param location : Vector position of the location, use field map
     */
    public void face(Vector location) {
        double radiansToTurn = Math.atan2(location.getY() - robotPos.getY(), location.getX() - robotPos.getX());
        int theta = gyroSensor.convertToDegrees(radiansToTurn);

        // Determine what angle to turn
        int tempRobotAngle = robotAngle > 180 ? 360 - robotAngle : robotAngle;
        if (tempRobotAngle * theta < 0) {
            if (Math.abs(tempRobotAngle) + Math.abs(theta) < 180) {
                if (tempRobotAngle > theta)
                    turn(Math.abs(tempRobotAngle) + Math.abs(theta), true);
                else
                    turn(Math.abs(tempRobotAngle) + Math.abs(theta), false);
            }
            else {
                if (tempRobotAngle > theta)
                    turn(360 - (Math.abs(theta) + Math.abs(tempRobotAngle)), false);
                else
                    turn(360 - (Math.abs(theta) + Math.abs(tempRobotAngle)), true);
            }
        }
        else if (tempRobotAngle != theta) {
            if (tempRobotAngle < theta)
                turn(theta - tempRobotAngle, false);
            else
                turn(tempRobotAngle - theta, true);
        }
    }

    /**
     * Turns a specific amount of degrees using an MRGyro
     * @param degrees : the amount of degrees to turn
     * @param right : if true, we turn right; if false, we turn left
     */
    public void turn(int degrees, boolean right) {
        gyroSensor.zero();

        int currAngle = Math.abs(gyroSensor.getAngle()); // Use getAngle() because it returns angle robot has turned from origin
        double pow = 1; // power applied to motors

        while (currAngle < degrees) {
            pow = (double) (degrees - currAngle) / degrees * 0.6 + 0.1;

            // Apply power to motors and update currAngle
            if (right)
                setPowers(pow, -pow, 0);
            else
                setPowers(-pow, pow, 0);
            currAngle = Math.abs(gyroSensor.getAngle());
        }
        setPowers(0, 0, 0);

        // Updates the robot angle based on turn
        if (right)
            setRobotAngle((360 + robotAngle - Math.abs(degrees)) % 360);
        else
            setRobotAngle((360 + robotAngle + Math.abs(degrees)) % 360);
    }



    // Accessor methods
    public double getLeftPow() {
        return leftMotor.getPower();
    }
    public double getRightPow() {
        return rightMotor.getPower();
    }
    public double getMiddlePow() {
        return middleMotor.getPower();
    }
    public Encoder getLeftEncoder() {
        return leftEncoder;
    }
    public Encoder getRightEncoder() {
        return rightEncoder;
    }
    public Encoder getMiddleEncoder() {
        return middleEncoder;
    }
    public MRGyro getGyro() {
        return gyroSensor;
    }
    public double getBoost() {
        return boost;
    }
}
