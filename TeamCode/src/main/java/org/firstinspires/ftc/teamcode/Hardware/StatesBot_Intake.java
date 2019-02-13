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


    //Auto functions


}
