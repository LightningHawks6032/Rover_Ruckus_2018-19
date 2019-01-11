package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.AutonomousData;

public class Robot2_Intake implements RobotHardware {
    // Hardware Components
    public DcMotor harvester; // Weird tubey thing that collects the minerals
    public DcMotor flipper; // Flips the collector
    public Encoder flipEncoder; // For managing the flipper position
    public DcMotor horizontalSlide; // Extends over crater
    public Encoder slideEncoder;

    private Gamepad gamepad;

    // Hardware Constants
    public final int FLIPPER_IN_ENCODER_VAL = 0;
    public final int FLIPPER_OUT_ENCODER_VAL = 700;

    // Booleans
    private boolean flippingIn = true; //should start as false if the flipper is in
    private boolean isFlipping = false;

    protected Robot2_Intake(DcMotor harvest, DcMotor flip, DcMotor hs, Gamepad manipsGamepad) {
        harvester = harvest;
        flipper = flip;
        horizontalSlide = hs;
        flipEncoder = new Encoder(flip, AutonomousData.NEVEREST_ENCODER, 0);
        slideEncoder = new Encoder(hs, AutonomousData.NEVEREST_ENCODER, 0);

        gamepad = manipsGamepad;
    }

    public void initHardware() {
        harvester.setDirection(DcMotor.Direction.REVERSE);
        flipper.setDirection(DcMotor.Direction.FORWARD);
        horizontalSlide.setDirection(DcMotor.Direction.FORWARD);
        flipEncoder.setup();
        slideEncoder.setup();
        flipper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void manageTeleOp() {
        collect();
        flip();
        manageSlide();
    }

    // Run the collector
    private void collect() {
        if (gamepad.a) {
            harvester.setPower(0.5);
        } else if (gamepad.y) {
            harvester.setPower(-0.5);
        } else {
            harvester.setPower(0);
        }
    }

    // Flip the collector
    private void flip() {
        // Use x to toggle between flipper in and flipper out

        // flipping inwards
        if (flippingIn && !isFlipping) {
            if (gamepad.x) {
                isFlipping = true;
            }
        } else if (flippingIn && isFlipping) {
            flipEncoder.setTarget(FLIPPER_IN_ENCODER_VAL);
            flipEncoder.runToPosition();
            flipper.setPower(0.35);
            if(!flipper.isBusy()){
                flipper.setPower(0);
                isFlipping = false;
                flippingIn = false;
            }

        }
        //flipping out
        if (!flippingIn && !isFlipping){
            if (gamepad.x) {
                isFlipping = true;
            }
        }else if(!flippingIn && isFlipping){
            flipEncoder.setTarget(FLIPPER_OUT_ENCODER_VAL);
            flipEncoder.runToPosition();
            flipper.setPower(0.35);
            if(!flipper.isBusy()){
                flipper.setPower(0);
                isFlipping = false;
                flippingIn = true;
            }
        }

    }



    // Manage horizontal slide
    private void manageSlide() {
        horizontalSlide.setPower(gamepad.left_stick_y);
    }



}
