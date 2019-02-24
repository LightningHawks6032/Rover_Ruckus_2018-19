package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.AutonomousData;
import org.firstinspires.ftc.teamcode.FieldMapping.FieldElement;
import org.firstinspires.ftc.teamcode.FieldMapping.Vector;

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

    // Navigation/Positional Components
    public Vector robotPos; // Position on field
    public double robotAngle; // Angle relative to (0, 0) on field
    private double initialIMUHeading; // IMU when the robot first hits the floor
    private double initialRobotAngle; // Manually-set robot angle when the robot first hits the floor

    private double wheelDiameter = 4.4;
    private double boost = 0.75;

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

    public void setRobotPos(Vector pos) {
        robotPos = pos;
    }
    public void setRobotAngle(double angle) {
        robotAngle = angle;
    }
    public void setInitialRobotAngle(double angle) {
        initialRobotAngle = angle;
    }
    public void setInitialIMUHeading() {
        initialIMUHeading = imu.getHeading();
    }

    public void initHardware() {
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        encoderSetup();
    }

    public void encoderSetup() {
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
        /*if (gamepad.left_stick_y == 0 && gamepad.right_stick_y == 0) {
            if (gamepad.right_trigger > 0) {
                manageStrafing(true);
            } else if (gamepad.left_trigger > 0) {
                manageStrafing(false);
            } else {
                setPowers(0, 0, 0, 0);
            }
        } else {
            if (gamepad.right_trigger > 0) {
                manageDiagonalStrafing(true);
            } else if (gamepad.left_trigger > 0) {
                manageDiagonalStrafing(false);
            } else {
                setPowers(-gamepad.left_stick_y * boost, -gamepad.right_stick_y * boost, -gamepad.left_stick_y * boost, -gamepad.right_stick_y * boost);
            }
        }*/

        if (gamepad.right_trigger > 0) {
            double pow = gamepad.right_trigger * boost;
            setPowers(pow, -pow, -pow, pow);
        } else if (gamepad.left_trigger > 0) {
            double pow = gamepad.left_trigger * boost;
            setPowers(-pow, pow, pow, -pow);
        } else {
            setPowers(-gamepad.left_stick_y * boost, -gamepad.right_stick_y * boost, -gamepad.left_stick_y * boost, -gamepad.right_stick_y * boost);
        }

        //applyBoost();
    }

    private void manageStrafing(boolean right) {
        double pow;
        if (right) {
            pow = gamepad.right_trigger * boost;
            setPowers(pow, -pow, -pow, pow);
        } else {
            pow = gamepad.left_trigger * boost;
            setPowers(-pow, pow, pow, -pow);
        }
    }

    private void manageDiagonalStrafing(boolean right) {
        double pow;
        if (right) {
            pow = -gamepad.right_stick_y * boost;
            if (pow > 0)
                setPowers(pow, 0, 0, pow);
            else
                setPowers(0, -pow, -pow, 0);
        } else {
            pow = -gamepad.left_stick_y * boost;
            if (pow > 0)
                setPowers(0, pow, pow, 0);
            else
                setPowers(-pow, 0, 0, -pow);
        }
    }

    private void applyBoost() {
        if (gamepad.left_bumper) {
            boost = 0.5;
        }
        if (gamepad.right_bumper) {
            boost = 1;
        }
    }

    /**
     * Robot drives forward or backward a set amount of linear distance using encoders
     * @param direction : forward (1) or backward (-1)
     * @param distance : linear distance in inches for the robot to drive over
     * @param pow : constant power at which the robot drives
     */
    public void driveDistance(int direction, double distance, double pow) throws InterruptedException {
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
        updateAngleFromIMU();
    }

    /**
     * Robot strafes left or right a set amount of linear distance using encoders
     * @param direction : right (1) or left (-1)
     * @param distance : linear distance in inches for the robot to strafe over
     * @param pow : constant power at which the robot strafes
     */
    public void strafeDistance(int direction, double distance, double pow) throws InterruptedException {
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
        updateAngleFromIMU();
    }

    public void strafeForTime(double pow, double seconds) throws InterruptedException {
        encoderSetup();
        setPowers(pow, -pow, -pow, pow);
        Thread.sleep((long) seconds * 1000);
        setPowers(0, 0, 0, 0);
    }

    public void goTo(Vector location, double pow) throws InterruptedException {
        face(location); // Turn to face location
        driveDistance(1, location.distanceFrom(robotPos), pow); // Drive to location
        updatePosAfterDrive(1);
        updateAngleFromIMU();
    }
    public void goToBackwards(Vector location, double pow) throws InterruptedException {
        double radiansToTurn = Math.atan2(location.getY() - robotPos.getY(), location.getX() - robotPos.getX());
        int theta = (int) MRGyro.convertToDegrees(radiansToTurn);

        if (theta <= 0) faceAngle(180 + theta);
        else faceAngle(360 - (180 + theta));
        driveDistance(-1, location.distanceFrom(robotPos), pow); // Drive to location
        updatePosAfterDrive(-1);
        updateAngleFromIMU();
    }

    /**
     * Robot turns to face certain location on field
     * @param location : Vector position of the location, use field map
     */
    public void face(Vector location) throws InterruptedException {
        double radiansToTurn = Math.atan2(location.getY() - robotPos.getY(), location.getX() - robotPos.getX());
        int theta = (int) MRGyro.convertToDegrees(radiansToTurn);

        faceAngle(theta);
    }

    public void face(FieldElement element) throws InterruptedException {
        face(AutonomousData.FIELD_MAP.get(element));
    }


    /**
     * Turns a specific amount of degrees using an MRGyro
     * @param degrees : the amount of degrees to turn
     * @param right : if true, we turn right; if false, we turn left
     */
    public void turn(int degrees, boolean right) throws InterruptedException {
        gyro.zero();
        encoderSetup();

        int currAngle = Math.abs(gyro.getAngle()); // Use getAngle() because it returns angle robot has turned from origin
        double startPow = 1;
        double pow; // power applied to motors
        double prop; // proportion of angle completed

        while (currAngle < degrees && autoRunning()) {
            prop = (double) (degrees - currAngle) / 90;/*currAngle / degrees;*/
            pow = startPow * Math.pow(prop, 1.8/*1.5*/) + 0.1;/*Math.pow(prop - 1, 2);*/
            if (pow > 1) pow = 1;

            // Apply power to motors and update currAngle
            if (right)
                setPowers(pow, -pow, pow, -pow);
            else
                setPowers(-pow, pow, -pow, pow);
            currAngle = Math.abs(gyro.getAngle());
        }
        setPowers(0, 0, 0, 0);

        // Updates the robot angle based on turn
        updateAngleFromIMU();
    }

    public void faceAngle(int theta) throws InterruptedException {
        updateAngleFromIMU();

        // Determine what angle to turn
        int tempRobotAngle = robotAngle > 180 ? -(360 - (int) Math.round(robotAngle)) : (int) Math.round(robotAngle);
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


    public void updatePosAfterDrive(int direction) {
        double theta = MRGyro.convertToRadians(getTemporaryAngle(direction == 1 ? 0 : 180));
        setRobotPos(robotPos.sum(new Vector(getAverageDist() * Math.cos(theta), getAverageDist() * Math.sin(theta))));
        resetEncoders();
    }
    public void updatePosAfterStrafe(int direction, boolean diagonal) {
        double theta = MRGyro.convertToRadians(getTemporaryAngle(-direction * 90));
        Vector newVec = robotPos.sum(new Vector(getAverageDist() * Math.cos(theta), getAverageDist() * Math.sin(theta)));
        setRobotPos(diagonal ? newVec.scale(Math.sqrt(2)) : newVec);
        resetEncoders();
    }
    public void updateAngleFromIMU() throws InterruptedException {
        Thread.sleep(200);
        setRobotAngle((360 + initialRobotAngle - (imu.getHeading() - initialIMUHeading)) % 360);
        autonomous.telemetry.addData("Angle", robotAngle);
        autonomous.telemetry.update();
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
        double sum = Math.abs(leftFrontEncoder.linDistance()) + Math.abs(rightFrontEncoder.linDistance()) +
                     Math.abs(leftBackEncoder.linDistance()) + Math.abs(rightBackEncoder.linDistance());
        return sum / 4;
    }
    private int getTemporaryAngle(int adjustmentFactor) {
        int RA = (360 + (int) Math.round(robotAngle + adjustmentFactor)) % 360;
        return RA > 180 ? -(360 - RA) : RA;
    }


    // Used to break all while loops when an opmode stops
    private boolean autoRunning() {
        return System.currentTimeMillis() - startTime <= AutonomousData.TIME_LIMIT && !autonomous.isStopRequested();
    }
}
