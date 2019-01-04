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


    protected Robot2_Outtake(DcMotor leftVert, DcMotor rightVert, Servo dump, Servo lPlate, Servo rPlate, Gamepad manipsGamepad) {
        leftVertical = leftVert;
        rightVertical = rightVert;
        leftVertEncoder = new Encoder(leftVert, AutonomousData.NEVEREST_ENCODER, 0);
        rightVertEncoder = new Encoder(leftVert, AutonomousData.NEVEREST_ENCODER, 0);

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
