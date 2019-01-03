package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot2_Outtake {
    // Hardware Components
    private DcMotor leftVertical;
    private DcMotor rightVertical;
    private Servo leftDumper;
    private Servo rightDumper;

    private Gamepad gamepad;

    public Robot2_Outtake(DcMotor leftVert, DcMotor rightVert, Servo lDump, Servo rDump, Gamepad manipsGamepad) {
        leftVertical = leftVert;
        rightVertical = rightVert;
        leftDumper = lDump;
        rightDumper = rDump;

        gamepad = manipsGamepad;
    }

    public void setupMotors() {
        leftVertical.setDirection(DcMotor.Direction.FORWARD);
        rightVertical.setDirection(DcMotor.Direction.FORWARD);
    }

    public void manageTeleOp(){
        lift();
        dump();
    }

    private void lift(){

    }

    private void dump(){

    }

}
