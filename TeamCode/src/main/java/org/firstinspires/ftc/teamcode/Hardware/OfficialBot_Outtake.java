package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.AutonomousData;

public class OfficialBot_Outtake implements RobotHardware {
    // Hardware Components
    public DcMotor leftVertical;
    public DcMotor rightVertical;
    public Encoder leftVertEncoder;
    public Encoder rightVertEncoder;
    //public Servo dumper;
    //public Servo leftPlate;
    //public Servo rightPlate;

    public Servo leftDumper;
    public Servo rightDumper;

    private Gamepad gamepad;

    // Servo constants
    public final double DUMPER_SERVO_DIFFERENCE = 0.7;
    public final double RIGHT_DUMPER_IN = 0.175,
            RIGHT_DUMPER_OUT = RIGHT_DUMPER_IN + DUMPER_SERVO_DIFFERENCE,
            LEFT_DUMPER_IN = 0.8,
            LEFT_DUMPER_OUT = LEFT_DUMPER_IN - DUMPER_SERVO_DIFFERENCE;
    /*
    public final double DUMPER_IN = 0,
            DUMPER_OUT = 0.8;
    */

    // Encoder constants (encoder setup happens at beginning of autonomous)
    public final int VERTICAL_SLIDE_MAX = 3470;
    public final int VERTICAL_SLIDE_MIN = 0;
    public final int LAND_ENCODER_VAL = 2450;

    // AUTO BASED VARIABLES
    private LinearOpMode autonomous = null; // stays null unless used in an auto
    private long startTime;

    protected OfficialBot_Outtake(DcMotor leftVert, DcMotor rightVert, Servo lServo, Servo rServo, Gamepad manipsGamepad) {
        leftVertical = leftVert;
        rightVertical = rightVert;
        leftVertEncoder = new Encoder(leftVert, AutonomousData.NEVEREST_40_ENCODER, 0);
        rightVertEncoder = new Encoder(rightVert, AutonomousData.NEVEREST_40_ENCODER, 0);

        leftDumper = lServo;
        rightDumper = rServo;

        gamepad = manipsGamepad;
    }

    public void initHardware() {
        leftVertical.setDirection(DcMotor.Direction.REVERSE);
        rightVertical.setDirection(DcMotor.Direction.REVERSE);
        leftVertEncoder.runWith();
        rightVertEncoder.runWith();

        /*
        dumper.setPosition(DUMPER_IN);
        leftPlate.setPosition(LEFT_PLATE_DOWN);
        rightPlate.setPosition(RIGHT_PLATE_DOWN);
        */
        leftDumper.setPosition(LEFT_DUMPER_IN);
        rightDumper.setPosition(RIGHT_DUMPER_IN);
    }

    public void setStartTime(long time) {
        startTime = time;
    }
    public void setAuto(LinearOpMode auto) {
        autonomous = auto;
    }

    public void manageTeleOp() {
        manageLifting();
        manageDumping();
    }

    // Manage vertical slide
    private boolean autoRetract = false; // Should the vertical slides be attempting to auto retract?
    private void manageLifting() {
        double pow = -gamepad.right_stick_y;
        int encoderVal = (leftVertEncoder.getEncoderCount() + rightVertEncoder.getEncoderCount()) / 2; // average

        if ((pow > 0 && encoderVal < VERTICAL_SLIDE_MAX) || (pow < 0 && encoderVal > VERTICAL_SLIDE_MIN)) {
            autoRetract = false;
            leftVertical.setPower(pow);
            rightVertical.setPower(pow);
        } else if (autoRetract && encoderVal > VERTICAL_SLIDE_MIN) {
            leftVertical.setPower(-1);
            rightVertical.setPower(-1);
        } else {
            leftVertical.setPower(0);
            rightVertical.setPower(0);
        }

        if (gamepad.right_bumper)
            autoRetract = true;

    }

    private void manageDumping() {
        /*
        if (gamepad.dpad_up)
            dumper.setPosition(DUMPER_OUT);
        else if (gamepad.dpad_down)
            dumper.setPosition(DUMPER_IN);

        if (gamepad.left_trigger > 0)
            rightPlate.setPosition(RIGHT_PLATE_UP);
        else
            rightPlate.setPosition(RIGHT_PLATE_DOWN);

        if (gamepad.right_trigger > 0)
            leftPlate.setPosition(LEFT_PLATE_UP);
        else
            leftPlate.setPosition(LEFT_PLATE_DOWN);
        */
        if (gamepad.dpad_up){
            leftDumper.setPosition(LEFT_DUMPER_IN);
            rightDumper.setPosition(RIGHT_DUMPER_IN);
        }
        else if (gamepad.dpad_down){
            leftDumper.setPosition(LEFT_DUMPER_OUT);
            rightDumper.setPosition(RIGHT_DUMPER_OUT);
        }


    }

    // Lands on the field for autonomous
    public void landOnField() {
        leftVertEncoder.reset();
        rightVertEncoder.reset();

        leftVertEncoder.runToPosition();
        rightVertEncoder.runToPosition();

        leftVertEncoder.setEncoderTarget(LAND_ENCODER_VAL);
        rightVertEncoder.setEncoderTarget(LAND_ENCODER_VAL);

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
        //dumper.setPosition(DUMPER_OUT);
        leftDumper.setPosition(LEFT_DUMPER_OUT);
        rightDumper.setPosition(RIGHT_DUMPER_OUT);
        Thread.sleep(1000);
        //dumper.setPosition(DUMPER_IN);
        leftDumper.setPosition(LEFT_DUMPER_IN);
        rightDumper.setPosition(RIGHT_DUMPER_IN);
    }


    // Used to break all while loops when an opmode stops
    private boolean autoRunning() {
        return System.currentTimeMillis() - startTime <= AutonomousData.TIME_LIMIT && !autonomous.isStopRequested();
    }
}
