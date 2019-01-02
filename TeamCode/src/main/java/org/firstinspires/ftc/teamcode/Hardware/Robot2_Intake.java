package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.AutonomousData;

public class Robot2_Intake {
    // Hardware Components
    public DcMotor collector; // Weird tubey thing that collects the minerals
    public DcMotor flipper; // Flips the collector
    public Encoder flipEncoder; // For managing the flipper position
    public DcMotor horizontalSlide; // Extends over crater

    private Gamepad gamepad;

    // Hardware Constants
    public final int FLIPPER_IN_ENCODER_VAL = 0;
    public final int FLIPPER_OUT_ENCODER_VAL = 600; // Needs to be changed with testing

    public Robot2_Intake(DcMotor collect, DcMotor flip, DcMotor hs, Gamepad manipsGamepad) {
        collector = collect;
        flipper = flip;
        horizontalSlide = hs;
        flipEncoder = new Encoder(flip, AutonomousData.NEVEREST_ENCODER, 0);

        gamepad = manipsGamepad;
    }

    public void setupMotors() {
        collector.setDirection(DcMotor.Direction.FORWARD);
        flipper.setDirection(DcMotor.Direction.FORWARD);
        horizontalSlide.setDirection(DcMotor.Direction.FORWARD);
        flipEncoder.setup();
    }

    public void manageTeleOp() {
        collect();
        flip();
        manageSlide();
    }

    // Run the collector
    private void collect() {
        if (gamepad.a) {
            collector.setPower(1);
        } else if (gamepad.b) {
            collector.setPower(-1);
        }
    }

    // Flip the collector
    private void flip() {

    }

    // Manage horizontal slide
    private void manageSlide() {

    }



}
