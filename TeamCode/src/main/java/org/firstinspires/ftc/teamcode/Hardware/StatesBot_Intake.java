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
    public RevRoboticsCoreHexMotor harvester;
    public Servo flipper;
    public DcMotor horizontalSlide;
    public Encoder slideEncoder;

    private Gamepad gamepad;

    //Positions for flipping and sliding
    public final int FLIPPER_IN_VAL = 1; //guesstimation
    public final int FLIPPER_OUT_VAL = 0; //guesstimation
    public final int HORIZONTAL_SLIDE_MAX = 2800;
    public final int HORIZONTAL_SLIDE_MIN = 0;

    // AUTO BASED VARIABLES
    private LinearOpMode autonomous = null; // stays null unless used in an auto
    private long startTime;

    protected StatesBot_Intake(RevRoboticsCoreHexMotor harvest, Servo flip, DcMotor hs, Gamepad manipsGamepad) {
        harvester = harvest;
        flipper = flip;
        horizontalSlide = hs;
        slideEncoder = new Encoder(hs, AutonomousData.NEVEREST_20_ENCODER, 0);

        gamepad = manipsGamepad;
    }

    public void initHardware() {
        //harvester.setDirection(RevRoboticsCoreHexMotor.Direction.FORWARD);
        // still haven't figured out core hex controls)
        horizontalSlide.setDirection(DcMotor.Direction.REVERSE);
        slideEncoder.runWith();
    }


    //TeleOp functions

    public void manageTeleOp() {
        manageHarvester();
        manageFlipper();
        manageSlide();
    }

    private void manageHarvester(){
        //Yet again foiled by hexcore motor
        if (gamepad.a) {
            //harvester.setPower(HARVESTER_POWER);
        } else if (gamepad.y) {
            //harvester.setPower(-HARVESTER_POWER);
        } else {
            //harvester.setPower(0);
        }

    }


    private void manageFlipper(){
        //this may not allow you to change direction mid-movement - micc
        if(gamepad.x && flipper.getPosition() == FLIPPER_IN_VAL){
            flipper.setPosition(FLIPPER_OUT_VAL);
        }else if(gamepad.x && flipper.getPosition() == FLIPPER_OUT_VAL){
            flipper.setPosition(FLIPPER_IN_VAL);
        }
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

    //Auto functions

    //nothing here as of now, as harvester is a core hex motor which we haven't figured out, and flipper is a servo
    //that can be accessed directly in auto without need for functions


}
