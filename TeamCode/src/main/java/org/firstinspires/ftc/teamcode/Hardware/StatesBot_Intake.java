package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.AutonomousData;

public class StatesBot_Intake {
    //Hardware Components
    public DcMotor harvester;
    public Servo leftFlipper;
    public Servo rightFlipper;
    public DcMotor horizontalSlide;
    public Encoder slideEncoder;

    private Gamepad gamepad;

    //Positions for flipping and sliding
    public final int RIGHT_FLIPPER_IN_VAL = 1, //guesstimation
    RIGHT_FLIPPER_OUT_VAL = 0, //guesstimation
    LEFT_FLIPPER_IN_VAL = 0, //guesstimation
    LEFT_FLIPPER_OUT_VAL = 1; //guesstimation
    public final int HORIZONTAL_SLIDE_MAX = 2800;
    public final int HORIZONTAL_SLIDE_MIN = 0;

    public final double HARVESTER_POWER = 1.0;

    // AUTO BASED VARIABLES
    private LinearOpMode autonomous = null; // stays null unless used in an auto
    private long startTime;

    protected StatesBot_Intake(DcMotor harvest, Servo rFlip, Servo lFlip, DcMotor hs, Gamepad manipsGamepad) {
        harvester = harvest;
        leftFlipper = lFlip;
        rightFlipper = rFlip;
        horizontalSlide = hs;
        slideEncoder = new Encoder(hs, AutonomousData.NEVEREST_20_ENCODER, 0);

        gamepad = manipsGamepad;
    }

    public void initHardware() {
        harvester.setDirection(DcMotor.Direction.FORWARD);
        horizontalSlide.setDirection(DcMotor.Direction.REVERSE);
        slideEncoder.runWith();
    }

    public void setStartTime(long time) {
        startTime = time;
    }
    public void setAuto(LinearOpMode auto) {
        autonomous = auto;
    }

    public void manageTeleOp(boolean slideLimiting) {
        manageHarvester();
        manageFlipper();
        manageSlide(slideLimiting);
    }

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

    // Flip the harvester
    private void manageFlipper() {
        // Use x to toggle between flipper in and flipper out
        togglePressed = gamepad.x;
        if (togglePressed && !toggleLastPressed) // Only change flipper if toggle button wasn't pressed last iteration of loop()
            flippingIn = !flippingIn;
        toggleLastPressed = togglePressed; // toggleLastPressed updated for the next iteration of loop()

        // Manage flip servo
        if (flippingIn) {
            leftFlipper.setPosition(LEFT_FLIPPER_IN_VAL);
            rightFlipper.setPosition(RIGHT_FLIPPER_IN_VAL);
        }else {
            rightFlipper.setPosition(RIGHT_FLIPPER_OUT_VAL);
            leftFlipper.setPosition(LEFT_FLIPPER_OUT_VAL);

        }
    }

    // Manage horizontal slide
    private boolean autoRetract = false; // Should the horizontal slide be attempting to auto retract?
    private void manageSlide(boolean slideLimiting) {
        double pow = -gamepad.left_stick_y;
        int encoderVal = slideEncoder.getEncoderCount();

        if ((pow > 0 && encoderVal < HORIZONTAL_SLIDE_MAX) || (pow < 0 && encoderVal > HORIZONTAL_SLIDE_MIN) || !slideLimiting) {
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



    //Auto functions

    //nothing here as of now, as harvester is a core hex motor which we haven't figured out, and flipper is a servo
    //that can be accessed directly in auto without need for functions


}
