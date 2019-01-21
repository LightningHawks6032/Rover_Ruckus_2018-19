package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

    // Power constants
    public final double HARVESTER_POWER = 0.7;

    // Encoder constants (encoder setup happens at beginning of autonomous)
    public final int FLIPPER_IN_ENCODER_VAL = 0;
    public final int FLIPPER_OUT_ENCODER_VAL = 7000;
    public final int HORIZONTAL_SLIDE_MAX = 2800;
    public final int HORIZONTAL_SLIDE_MIN = 0;

    // AUTO BASED VARIABLES
    private LinearOpMode autonomous = null; // stays null unless used in an auto
    private long startTime;

    protected OfficialBot_Intake(DcMotor harvest, DcMotor flip, DcMotor hs, Gamepad manipsGamepad) {
        harvester = harvest;
        flipper = flip;
        horizontalSlide = hs;
        flipEncoder = new Encoder(flip, AutonomousData.NEVEREST_20_ENCODER, 0);
        slideEncoder = new Encoder(hs, AutonomousData.NEVEREST_20_ENCODER, 0);

        gamepad = manipsGamepad;
    }

    public void initHardware() {
        harvester.setDirection(DcMotor.Direction.FORWARD);
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
            harvester.setPower(HARVESTER_POWER);
        } else if (gamepad.y) {
            harvester.setPower(-HARVESTER_POWER);
        } else {
            harvester.setPower(0);
        }
    }

    // Booleans to manage flipping for tele-op
    private boolean flippingIn = true; // Should the flipper be flipping inward? (i.e. was the last command to flip inward?)
    private boolean togglePressed = false; // Is the toggle button currently pressed?
    private boolean toggleLastPressed = false; // Was the toggle button pressed last iteration of loop()?

    // Flip the collector
    private void manageFlipper() {
        // Use x to toggle between flipper in and flipper out
        togglePressed = gamepad.x;
        if (togglePressed && !toggleLastPressed) // Only change flipper if toggle button wasn't pressed last iteration of loop()
            flippingIn = !flippingIn;
        toggleLastPressed = togglePressed; // toggleLastPressed updated for the next iteration of loop()

        // Manage flip motor
        if (flippingIn && flipEncoder.getEncoderCount() > FLIPPER_IN_ENCODER_VAL)
            flipper.setPower(-1);
        else if (!flippingIn && flipEncoder.getEncoderCount() < FLIPPER_OUT_ENCODER_VAL)
            flipper.setPower(1);
        else
            flipper.setPower(0);
    }

    // Manage horizontal slide
    private boolean autoRetract = false; // Should the horizontal slide be attempting to auto retract?
    private void manageSlide() {
        double pow = -gamepad.left_stick_y;
        int encoderVal = slideEncoder.getEncoderCount();

        if ((pow > 0 && encoderVal < HORIZONTAL_SLIDE_MAX) || (pow < 0 && encoderVal > HORIZONTAL_SLIDE_MIN)) {
            autoRetract = false;
            horizontalSlide.setPower(pow);
        } else if (autoRetract && encoderVal > HORIZONTAL_SLIDE_MIN) {
            horizontalSlide.setPower(-1);
        } else {
            horizontalSlide.setPower(0);
        }

        if (gamepad.left_bumper)
            autoRetract = true;

    }

    // Harvesting in autonomous
    public void harvest() {
        harvester.setPower(HARVESTER_POWER);
    }
    public void putMineralsInDumper() throws InterruptedException {
        harvester.setPower(-HARVESTER_POWER);
        Thread.sleep(2000);
        stopHarvesting();
    }
    public void stopHarvesting() {
        harvester.setPower(0);
    }

    // Utilizing intake flipper in autonomous
    public void flipIn() {
        flipEncoder.runToPosition();
        flipEncoder.setEncoderTarget(FLIPPER_IN_ENCODER_VAL);
        flipper.setPower(-1);
        while (flipper.isBusy() && autoRunning()) {
            // WAIT - Motor is busy
        }
        flipper.setPower(0);
    }
    public void flipOut() {
        flipEncoder.runToPosition();
        flipEncoder.setEncoderTarget(FLIPPER_OUT_ENCODER_VAL);
        flipper.setPower(1);
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
