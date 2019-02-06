package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.AutonomousData;

public class MecanumWheelDrive implements RobotHardware {
    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor leftBack;
    public DcMotor rightBack;
    public Encoder leftFrontEncoder;
    public Encoder rightFrontEncoder;
    public Encoder leftBackEncoder;
    public Encoder rightBackEncoder;
    public ExpansionHubIMU imu;
    public MRGyro gyro;
    private Gamepad gamepad;

    private int wheelDiameter = 4;


    // AUTO BASED VARIABLES
    private LinearOpMode autonomous = null; // stays null unless used in an auto
    public long startTime;

    public MecanumWheelDrive(DcMotor lf, DcMotor rf, DcMotor lb, DcMotor rb, ExpansionHubIMU hubIMU, MRGyro gyroSensor, Gamepad gamepad) {
        leftFront = lf;
        rightFront = rf;
        leftBack = lb;
        rightBack = rb;
        imu = hubIMU;
        gyro = gyroSensor;
        this.gamepad = gamepad;

        leftFrontEncoder = new Encoder(lf, AutonomousData.NEVEREST_20_ENCODER, wheelDiameter);
        rightFrontEncoder = new Encoder(rf, AutonomousData.NEVEREST_20_ENCODER, wheelDiameter);
        leftBackEncoder = new Encoder(lb, AutonomousData.NEVEREST_20_ENCODER, wheelDiameter);
        rightBackEncoder = new Encoder(rb, AutonomousData.NEVEREST_20_ENCODER, wheelDiameter);
    }

    public void setStartTime(long time) {
        startTime = time;
    }
    public void setAuto(LinearOpMode auto) {
        autonomous = auto;
    }

    public void initHardware() {
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        encoderSetup();
    }

    private void encoderSetup() {
        leftFrontEncoder.setup();
        rightFrontEncoder.setup();
        leftBackEncoder.setup();
        rightBackEncoder.setup();
    }

    private void resetEncoders() {
        leftFrontEncoder.reset();
        rightFrontEncoder.reset();
        leftBackEncoder.reset();
        rightBackEncoder.reset();
    }

    private void runWithoutEncoders() {
        leftFrontEncoder.runWithout();
        rightFrontEncoder.runWithout();
        leftBackEncoder.runWithout();
        rightBackEncoder.runWithout();
    }

    private void runEncodersToPosition() {
        leftFrontEncoder.runToPosition();
        rightFrontEncoder.runToPosition();
        leftBackEncoder.runToPosition();
        rightBackEncoder.runToPosition();
    }

    // Shortcut method for setting the power of the motors
    public void setPowers(double lfp, double rfp, double lbp, double rbp) {
        leftFront.setPower(lfp);
        rightFront.setPower(rfp);
        leftBack.setPower(lbp);
        rightBack.setPower(rbp);
    }

    public void manageTeleOp() {
        if (gamepad.left_stick_y == 0 && gamepad.right_stick_y == 0) {
            if (gamepad.right_trigger > 0) {
                manageStrafing(true);
            } else if (gamepad.left_trigger > 0) {
                manageStrafing(false);
            }
        } else {
            if (gamepad.right_trigger > 0) {
                manageDiagonalStrafing(true);
            } else if (gamepad.left_trigger > 0) {
                manageDiagonalStrafing(false);
            } else {
                setPowers(-gamepad.left_stick_y, -gamepad.right_stick_y, -gamepad.left_stick_y, -gamepad.right_stick_y);
            }
        }
    }

    private void manageStrafing(boolean right) {
        double pow;
        if (right) {
            pow = gamepad.right_trigger;
            setPowers(pow, -pow, -pow, pow);
        } else {
            pow = gamepad.left_trigger;
            setPowers(-pow, pow, pow, -pow);
        }
    }

    private void manageDiagonalStrafing(boolean right) {
        double pow;
        if (right) {
            pow = -gamepad.right_stick_y;
            if (pow > 0)
                setPowers(pow, 0, 0, pow);
            else
                setPowers(0, -pow, -pow, 0);
        } else {
            pow = -gamepad.left_stick_y;
            if (pow > 0)
                setPowers(0, pow, pow, 0);
            else
                setPowers(-pow, 0, 0, -pow);
        }
    }

    /**
     * Robot drives forward or backward a set amount of linear distance using encoders
     * @param direction : forward (1) or backward (-1)
     * @param distance : linear distance in inches for the robot to drive over
     * @param pow : constant power at which the robot drives
     */
    public void driveDistance(int direction, double distance, double pow) {
        resetEncoders();
        runEncodersToPosition();

        leftFrontEncoder.setTarget(direction * distance);
        rightFrontEncoder.setTarget(direction * distance);
        leftBackEncoder.setTarget(direction * distance);
        rightBackEncoder.setTarget(direction * distance);

        setPowers(direction * pow, direction * pow, direction * pow, direction * pow);

        while (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy() && autoRunning()) {
            // WAIT - Motors are busy
        }

        setPowers(0, 0, 0, 0);
        runWithoutEncoders();
        //updateAngleFromIMU();
    }

    /**
     * Robot strafes left or right a set amount of linear distance using encoders
     * @param direction : right (1) or left (-1)
     * @param distance : linear distance in inches for the robot to strafe over
     * @param pow : constant power at which the robot strafes
     */
    public void strafeDistance(int direction, double distance, double pow) {
        resetEncoders();
        runEncodersToPosition();

        leftFrontEncoder.setTarget(direction * distance);
        rightFrontEncoder.setTarget(-direction * distance);
        leftBackEncoder.setTarget(-direction * distance);
        rightBackEncoder.setTarget(direction * distance);

        setPowers(pow, -pow, -pow, pow);

        while (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy() && autoRunning()) {
            // WAIT - Motor is busy
        }

        setPowers(0, 0, 0, 0);

        runWithoutEncoders();
        //updateAngleFromIMU();
    }

    /**
     * Turns a specific amount of degrees using an MRGyro
     * @param degrees : the amount of degrees to turn
     * @param right : if true, we turn right; if false, we turn left
     */
    public void turn(int degrees, boolean right) {
        gyro.zero();
        encoderSetup();

        int currAngle = Math.abs(gyro.getAngle()); // Use getAngle() because it returns angle robot has turned from origin
        double startPow = 1;
        double pow; // power applied to motors
        double prop; // proportion of angle completed

        while (currAngle < degrees && autoRunning()) {
            prop = (double) currAngle / degrees;
            pow = -startPow * Math.pow(prop - 1, 3);

            // Apply power to motors and update currAngle
            if (right)
                setPowers(pow, -pow, pow, -pow);
            else
                setPowers(-pow, pow, -pow, pow);
            currAngle = Math.abs(gyro.getAngle());
        }
        setPowers(0, 0, 0, 0);

        // Updates the robot angle based on turn
        //updateAngleFromIMU();
    }


    // Accessor methods
    public double getLeftFrontPower() {
        return leftFront.getPower();
    }
    public double getRightFrontPower() {
        return rightFront.getPower();
    }
    public double getLeftBackPower() {
        return leftBack.getPower();
    }
    public double getRightBackPower() {
        return rightBack.getPower();
    }
    public double getAverageDist() {
        double sum = leftFrontEncoder.linDistance() + rightFrontEncoder.linDistance() + leftBackEncoder.linDistance() + rightBackEncoder.linDistance();
        return sum / 4;
    }


    // Used to break all while loops when an opmode stops
    private boolean autoRunning() {
        return System.currentTimeMillis() - startTime <= AutonomousData.TIME_LIMIT && !autonomous.isStopRequested();
    }
}
