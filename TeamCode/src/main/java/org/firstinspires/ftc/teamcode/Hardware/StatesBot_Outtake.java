package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.AutonomousData;

public class StatesBot_Outtake implements RobotHardware {
    // Hardware Components
    public DcMotor leftVertical;
    public DcMotor rightVertical;
    public Encoder leftVertEncoder;
    public Encoder rightVertEncoder;
    public Servo leftDumper;
    public Servo rightDumper;

    private Gamepad gamepad;

    // Servo constants
    public final double DUMPER_SERVO_DIFFERENCE = 0.6;
    public final double RIGHT_DUMPER_IN = 0.65,
            RIGHT_DUMPER_OUT = RIGHT_DUMPER_IN - DUMPER_SERVO_DIFFERENCE,
            LEFT_DUMPER_IN = 0.1,
            LEFT_DUMPER_OUT = LEFT_DUMPER_IN + DUMPER_SERVO_DIFFERENCE;

    // Encoder constants (encoder setup happens at beginning of autonomous)
    public final int VERTICAL_SLIDE_MAX = 3300;
    public final int VERTICAL_SLIDE_MIN = 0;
    public final int HANG_ENCODER_VAL = 2450;

    // AUTO BASED VARIABLES
    private LinearOpMode autonomous = null; // stays null unless used in an auto
    private long startTime;

    protected StatesBot_Outtake(DcMotor leftVert, DcMotor rightVert, Servo lServo, Servo rServo, Gamepad manipsGamepad) {
        leftVertical = leftVert;
        rightVertical = rightVert;
        leftVertEncoder = new Encoder(leftVert, AutonomousData.NEVEREST_40_ENCODER, 0);
        rightVertEncoder = new Encoder(rightVert, AutonomousData.NEVEREST_40_ENCODER, 0);

        leftDumper = lServo;
        rightDumper = rServo;

        gamepad = manipsGamepad;
    }

    public void initHardware() {
        leftVertical.setDirection(DcMotor.Direction.FORWARD);
        rightVertical.setDirection(DcMotor.Direction.FORWARD);
        leftVertEncoder.runWith();
        rightVertEncoder.runWith();
        leftDumper.setPosition(LEFT_DUMPER_IN);
        rightDumper.setPosition(RIGHT_DUMPER_IN);
    }

    public void setStartTime(long time) {
        startTime = time;
    }
    public void setAuto(LinearOpMode auto) {
        autonomous = auto;
    }

    public void manageTeleOp(boolean slideLimiting) {
        manageLifting(slideLimiting);
        //manageDumping();
    }

    // Manage vertical slide
    private boolean autoRetract = false; // Should the vertical slides be attempting to auto retract?
    private boolean autoExtend = false; // Should the vertical slides be attempting to auto extend?
    private void manageLifting(boolean slideLimiting) {
        double pow = -gamepad.right_stick_y;
        int encoderVal = (leftVertEncoder.getEncoderCount() + rightVertEncoder.getEncoderCount()) / 2; // average

        if ((pow > 0 && encoderVal < VERTICAL_SLIDE_MAX) || (pow < 0 && encoderVal > VERTICAL_SLIDE_MIN) || !slideLimiting) {
            autoRetract = false;
            autoExtend = false;
            leftVertical.setPower(pow);
            rightVertical.setPower(pow);
        } else if (gamepad.right_trigger > 0 && (encoderVal < HANG_ENCODER_VAL - 30 || encoderVal > HANG_ENCODER_VAL + 30)) {
            autoRetract = false;
            autoExtend = false;
            double hangPow = encoderVal < HANG_ENCODER_VAL ? 0.5 : -0.5;
            leftVertical.setPower(hangPow);
            rightVertical.setPower(hangPow);
        } else if (autoRetract && encoderVal > VERTICAL_SLIDE_MIN) {
            leftVertical.setPower(-1);
            rightVertical.setPower(-1);
        } else if (autoExtend && encoderVal < VERTICAL_SLIDE_MAX) {
            leftVertical.setPower(1);
            rightVertical.setPower(1);
        } else {
            leftVertical.setPower(0);
            rightVertical.setPower(0);
            autoRetract = false;
            autoExtend = false;
        }

        if (gamepad.left_bumper) {
            autoRetract = true;
            autoExtend = false;
        } else if (gamepad.right_bumper) {
            autoExtend = true;
            autoRetract = false;
        }
    }

    // Changed to driver controller
    /*private void manageDumping() {
        if (gamepad.dpad_down) {
            leftDumper.setPosition(LEFT_DUMPER_IN);
            rightDumper.setPosition(RIGHT_DUMPER_IN);
        }
        else if (gamepad.dpad_up) {
            leftDumper.setPosition(LEFT_DUMPER_OUT);
            rightDumper.setPosition(RIGHT_DUMPER_OUT);
        }
    }*/

    // Vertical slide is brought up/down
    public void verticalSlideUp() {
        leftVertEncoder.runToPosition();
        rightVertEncoder.runToPosition();

        leftVertEncoder.setEncoderTarget(VERTICAL_SLIDE_MAX);
        rightVertEncoder.setEncoderTarget(VERTICAL_SLIDE_MAX);

        leftVertical.setPower(1);
        rightVertical.setPower(1);
        while (leftVertical.isBusy() && rightVertical.isBusy() && autoRunning()) {
            // WAIT - Motor is busy
        }
        leftVertical.setPower(0);
        rightVertical.setPower(0);

        leftVertEncoder.runWithout();
        rightVertEncoder.runWithout();
    }
    public void verticalSlideDown() {
        leftVertEncoder.runToPosition();
        rightVertEncoder.runToPosition();

        leftVertEncoder.setEncoderTarget(VERTICAL_SLIDE_MIN);
        rightVertEncoder.setEncoderTarget(VERTICAL_SLIDE_MIN);

        leftVertical.setPower(-1);
        rightVertical.setPower(-1);
        while (leftVertical.isBusy() && rightVertical.isBusy() && autoRunning()) {
            // WAIT - Motor is busy
        }
        leftVertical.setPower(0);
        rightVertical.setPower(0);

        leftVertEncoder.runWithout();
        rightVertEncoder.runWithout();
    }

    // Dumping in autonomous
    public void dump() throws InterruptedException {
        leftDumper.setPosition(LEFT_DUMPER_OUT);
        rightDumper.setPosition(RIGHT_DUMPER_OUT);
        Thread.sleep(1000);
        leftDumper.setPosition(LEFT_DUMPER_IN);
        rightDumper.setPosition(RIGHT_DUMPER_IN);
    }

    // Used to break all while loops when an opmode stops
    private boolean autoRunning() {
        return System.currentTimeMillis() - startTime <= AutonomousData.TIME_LIMIT && !autonomous.isStopRequested();
    }
}
