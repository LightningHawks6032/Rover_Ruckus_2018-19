package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class OmniSlideDrive {
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor middleMotor;
    private Encoder leftEncoder;
    private Encoder rightEncoder;
    private Encoder middleEncoder;
    private Gamepad gamepad;
    private double boost;

    // Constants to regulate maximum power
    private final double MAX_DRIVE_POWER = 1;
    private final double MAX_MIDDLE_POWER = 1;

    public OmniSlideDrive(DcMotor lm, DcMotor rm, DcMotor mm, Gamepad gamepad, double wheelDiam) {
        leftMotor = lm;
        rightMotor = rm;
        middleMotor = mm;
        this.gamepad = gamepad;
        leftEncoder = new Encoder(lm, "Neverest", wheelDiam);
        rightEncoder = new Encoder(rm, "Neverest", wheelDiam);
        middleEncoder = new Encoder(mm, "Neverest", wheelDiam);

        boost = 0.7;
    }

    // Shortcut method for setting the power of the left drive, right drive, and middle drive
    private void setPowers(double lp, double rp, double mp) {
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
    public double getBoost() {
        return boost;
    }
}
