package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.AutonomousData;

public class OfficialBot_Intake implements RobotHardware {
    // Hardware Components
    public DcMotor harvester; // Weird tubey thing that collects the minerals
    public DcMotor flipper; // Flips the collector
    public Encoder flipEncoder; // For managing the flipper position
    public DcMotor horizontalSlide; // Extends over crater
    public Encoder slideEncoder;

    private Gamepad gamepad;

    // Encoder constants (encoder setup happens at beginning of autonomous)
    public final int FLIPPER_IN_ENCODER_VAL = 0;
    public final int FLIPPER_OUT_ENCODER_VAL = 850;
    public final int HORIZONTAL_SLIDE_MAX = 2800;
    public final int HORIZONTAL_SLIDE_MIN = 0;

    // AUTO BASED VARIABLES
    private LinearOpMode autonomous = null; // stays null unless used in an auto
    private long startTime;

    protected OfficialBot_Intake(DcMotor harvest, DcMotor flip, DcMotor hs, Gamepad manipsGamepad) {
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
        horizontalSlide.setDirection(DcMotor.Direction.REVERSE);
        flipEncoder.runWith();
        slideEncoder.runWith();
    }

    public void setStartTime(long time) {
        startTime = time;
    }
    public void setAuto(LinearOpMode auto) {
        autonomous = auto;
    }

    public void manageTeleOp() {
        manageHarvester();
        manageFlipper();
        manageSlide();
    }

    // Run the collector
    private void manageHarvester() {
        if (gamepad.a) {
            harvester.setPower(0.75);
        } else if (gamepad.y) {
            harvester.setPower(-0.75);
        } else {
            harvester.setPower(0);
        }
    }

    // Booleans to manage flipping for tele-op
    public boolean flippingIn = true; // Should the flipper be flipping inward? (i.e. was the last command to flip inward?)
    public boolean togglePressed = false; // Is the toggle button currently pressed?
    public boolean toggleLastPressed = false; // Was the toggle button pressed last iteration of loop()?

    // Flip the collector
    private void manageFlipper() {
        // Use x to toggle between flipper in and flipper out
        togglePressed = gamepad.x;
        if (togglePressed && !toggleLastPressed) // Only change flipper if toggle button wasn't pressed last iteration of loop()
            flippingIn = !flippingIn;
        toggleLastPressed = togglePressed; // toggleLastPressed updated for the next iteration of loop()

        // Manage flip motor
        if (flippingIn && flipEncoder.getEncoderCount() > FLIPPER_IN_ENCODER_VAL)
            flipper.setPower(-0.4);
        else if (!flippingIn && flipEncoder.getEncoderCount() < FLIPPER_OUT_ENCODER_VAL)
            flipper.setPower(0.4);
        else
            flipper.setPower(0);
    }


    // Manage horizontal slide
    private void manageSlide() {
        double pow = -gamepad.left_stick_y;
        int encoderVal = slideEncoder.getEncoderCount();

        if ((pow > 0 && encoderVal < HORIZONTAL_SLIDE_MAX) || (pow < 0 && encoderVal > HORIZONTAL_SLIDE_MIN)) {
            horizontalSlide.setPower(pow);
        } else {
            horizontalSlide.setPower(0);
        }
    }


    public void flipOut() {
        flipEncoder.runToPosition();
        flipEncoder.setEncoderTarget(FLIPPER_OUT_ENCODER_VAL);
        flipper.setPower(0.4);
        while (flipper.isBusy() && autoRunning()) {
            // WAIT - Motor is busy
        }
        flipper.setPower(0);
    }

    // Used to break all while loops when an opmode stops
    private boolean autoRunning() {
        return System.currentTimeMillis() - startTime <= AutonomousData.TIME_LIMIT && !autonomous.isStopRequested();
    }

}
