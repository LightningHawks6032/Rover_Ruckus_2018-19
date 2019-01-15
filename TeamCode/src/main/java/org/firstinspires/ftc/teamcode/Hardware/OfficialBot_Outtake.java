package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.AutonomousData;

public class OfficialBot_Outtake implements RobotHardware{
    // Hardware Components
    public DcMotor leftVertical;
    public DcMotor rightVertical;
    public Encoder leftVertEncoder;
    public Encoder rightVertEncoder;
    public Servo dumper;
    public Servo leftPlate;
    public Servo rightPlate;

    private Gamepad gamepad;

    // Servo constants
    public final double LEFT_PLATE_UP = 0.5,
            LEFT_PLATE_DOWN = 0.9,
            RIGHT_PLATE_UP = 0.8,
            RIGHT_PLATE_DOWN = 0.4;

    public final double DUMPER_IN = 0.1,
            DUMPER_OUT = 0.9;

    // Encoder constants (encoder setup happens at beginning of autonomous)
    public final int VERTICAL_SLIDE_MAX = 3470;
    public final int VERTICAL_SLIDE_MIN = 0;
    public final int LAND_ENCODER_VAL = 2840;

    // AUTO BASED VARIABLES
    private LinearOpMode autonomous = null; // stays null unless used in an auto
    private long startTime;

    protected OfficialBot_Outtake(DcMotor leftVert, DcMotor rightVert, Servo dump, Servo lPlate, Servo rPlate, Gamepad manipsGamepad) {
        leftVertical = leftVert;
        rightVertical = rightVert;
        leftVertEncoder = new Encoder(leftVert, AutonomousData.NEVEREST_ENCODER, 0);
        rightVertEncoder = new Encoder(rightVert, AutonomousData.NEVEREST_ENCODER, 0);

        dumper = dump;
        leftPlate = lPlate;
        rightPlate = rPlate;

        gamepad = manipsGamepad;
    }

    public void initHardware(boolean resetEncoders) {
        leftVertical.setDirection(DcMotor.Direction.REVERSE);
        rightVertical.setDirection(DcMotor.Direction.REVERSE);

        if (resetEncoders) {
            leftVertEncoder.setup();
            rightVertEncoder.setup();
        }
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

    private void manageLifting() {
        double pow = -gamepad.right_stick_y;
        int encoderVal = (leftVertEncoder.getEncoderCount() + rightVertEncoder.getEncoderCount()) / 2; // average

        if ((pow > 0 && encoderVal < VERTICAL_SLIDE_MAX) || (pow < 0 && encoderVal > VERTICAL_SLIDE_MIN)) {
            leftVertical.setPower(pow);
            rightVertical.setPower(pow);
        } else {
            leftVertical.setPower(0);
            rightVertical.setPower(0);
        }
    }

    private void manageDumping() {
        if (gamepad.dpad_up)
            dumper.setPosition(DUMPER_OUT);
        else if (gamepad.dpad_down)
            dumper.setPosition(DUMPER_IN);

        if (gamepad.left_trigger > 0)
            leftPlate.setPosition(RIGHT_PLATE_UP);
        else
            leftPlate.setPosition(RIGHT_PLATE_DOWN);

        if (gamepad.right_trigger > 0)
            rightPlate.setPosition(LEFT_PLATE_UP);
        else
            rightPlate.setPosition(LEFT_PLATE_DOWN);
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
    }


    // Used to break all while loops when an opmode stops
    private boolean autoRunning() {
        return System.currentTimeMillis() - startTime <= AutonomousData.TIME_LIMIT && !autonomous.isStopRequested();
    }
}
