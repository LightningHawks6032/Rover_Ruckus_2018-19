package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.AutonomousData;

public class Robot2_Outtake {
    // Hardware Components
    public DcMotor leftVertical;
    public DcMotor rightVertical;
    public Encoder leftVertEncoder;
    public Encoder rightVertEncoder;
    public Servo leftDumper;
    public Servo rightDumper;

    private Gamepad gamepad;


    protected Robot2_Outtake(DcMotor leftVert, DcMotor rightVert, Servo lDump, Servo rDump, Gamepad manipsGamepad) {
        leftVertical = leftVert;
        rightVertical = rightVert;
        leftVertEncoder = new Encoder(leftVert, AutonomousData.NEVEREST_ENCODER, 0);
        rightVertEncoder = new Encoder(leftVert, AutonomousData.NEVEREST_ENCODER, 0);

        leftDumper = lDump;
        rightDumper = rDump;

        gamepad = manipsGamepad;
    }

    public void setupMotors() {
        leftVertical.setDirection(DcMotor.Direction.FORWARD);
        rightVertical.setDirection(DcMotor.Direction.FORWARD);

        leftVertEncoder.setup();
        rightVertEncoder.setup();
    }

    public void manageTeleOp(){
        lift();
        dump();
    }

    private void lift(){
        double pow = gamepad.left_stick_y;
        leftVertical.setPower(pow);
        rightVertical.setPower(pow);
    }

    private void dump(){

    }

}
