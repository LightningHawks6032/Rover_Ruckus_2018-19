package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.AutonomousData;

public class Robot2_Outtake implements RobotHardware{
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
    public final double LEFT_PLATE_UP = 0.4,
            LEFT_PLATE_DOWN = 0.9,
            RIGHT_PLATE_UP = 0.9,
            RIGHT_PLATE_DOWN = 0.4;

    public final double DUMPER_IN = 0,
            DUMPER_OUT = 1;

    // Encoder constants
    public final int VERTICAL_SLIDE_MAX = -3470; // this is negative because the encoders on the vertical slide run opposite
    public final int VERTICAL_SLIDE_MIN = 0;
    public final int LAND_ENCODER_VAL = -2840;


    protected Robot2_Outtake(DcMotor leftVert, DcMotor rightVert, Servo dump, Servo lPlate, Servo rPlate, Gamepad manipsGamepad) {
        leftVertical = leftVert;
        rightVertical = rightVert;
        leftVertEncoder = new Encoder(leftVert, AutonomousData.NEVEREST_ENCODER, 0);
        rightVertEncoder = new Encoder(rightVert, AutonomousData.NEVEREST_ENCODER, 0);

        dumper = dump;
        leftPlate = lPlate;
        rightPlate = rPlate;

        gamepad = manipsGamepad;
    }

    public void initHardware() {
        leftVertical.setDirection(DcMotor.Direction.FORWARD);
        rightVertical.setDirection(DcMotor.Direction.FORWARD);

        leftVertEncoder.setup();
        rightVertEncoder.setup();
    }

    public void manageTeleOp() {
        lift();
        dump();
    }

    private void lift() {
        // potential issue; running w/ encoder w/o using them
        // potential fix: set target based on if pow is negative or positive
        leftVertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightVertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double pow = gamepad.right_stick_y;
        leftVertical.setPower(pow);
        rightVertical.setPower(pow);
        // highest position on left
        if(leftVertEncoder.getEncoderCount() >= VERTICAL_SLIDE_MAX && pow > 0){
            leftVertical.setPower(0);
        }
        // lowest position on left
        if(leftVertEncoder.getEncoderCount() <= VERTICAL_SLIDE_MIN && pow < 0){
            leftVertical.setPower(0);
        }
        // highest position on right
        if(rightVertEncoder.getEncoderCount() >= VERTICAL_SLIDE_MAX && pow > 0){
            rightVertical.setPower(0);
        }
        // lowest position on right
        if(rightVertEncoder.getEncoderCount() <= VERTICAL_SLIDE_MIN && pow < 0){
            rightVertical.setPower(0);
        }
    }

    private void dump() {
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

}
