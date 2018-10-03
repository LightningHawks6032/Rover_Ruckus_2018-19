package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name="Robot1 TeleOp", group="Iterative Opmode")
public class Robot1_TeleOp extends OpMode {

    Robot1_Hardware hardware;

    //Power variables
    private double leftPower,
        rightPower,
        boost,
        middlePower;

    // Max Power control variables
    private double MAX_MIDDLE_POWER = 0.8;
    private double MAX_DRIVE_POWER = 0.8;

    private int driveMode;

    public void init() {
        //Initialize hardware
        hardware = new Robot1_Hardware(hardwareMap);
        hardware.initHardware();

        //Assign variables
        boost = 0.7;
        rightPower = 0;
        leftPower = 0;
        driveMode = 1;
    }

    public void loop() {
        manageDrivetrain();
        applyBoost();
        /*telemetry.addData("Left Stick", gamepad1.left_stick_y);
        telemetry.addData("Right Stick", gamepad1.right_stick_y);
        telemetry.update();*/
    }

    private void applyBoost() {
        if (gamepad1.x)
            boost = 1;
    }

    // Shortcut method for setting the power of the left drive, right drive, and middle drive
    private void setPowers(double lp, double rp, double mp) {
        leftPower = lp;
        rightPower = rp;
        middlePower = mp;
    }

    private void basicDrive() {
        //drive1 controls (slide drive)
        if (gamepad1.left_trigger > 0) {
            setPowers(0, 0, -MAX_MIDDLE_POWER); // strafe left
        } else if (gamepad1.right_trigger > 0) {
            setPowers(0, 0, MAX_MIDDLE_POWER); // strafe right
        } else {
            // motor power = joysticks
            setPowers(-gamepad1.left_stick_y * MAX_DRIVE_POWER, -gamepad1.right_stick_y * MAX_DRIVE_POWER, 0);
        }
    }


    private void planetaryDrive() {
        // drive2 controls (planetary drive)
        if (gamepad1.left_trigger == 0 && gamepad1.right_trigger == 0) {
            //Moving forward, backward, left, right if not touching triggers
            setPowers(-gamepad1.left_stick_y * MAX_DRIVE_POWER,
                      -gamepad1.left_stick_y * MAX_DRIVE_POWER,
                      gamepad1.left_stick_x * MAX_MIDDLE_POWER);
        } else {
            // Turning
            if (gamepad1.left_trigger > 0) {
                setPowers(-MAX_DRIVE_POWER, MAX_DRIVE_POWER, 0);
            } else if (gamepad1.right_trigger > 0) {
                setPowers(MAX_DRIVE_POWER, -MAX_DRIVE_POWER, 0);
            }

        }
    }

    private void manageMode() {
        if (gamepad1.left_bumper)
            driveMode = 1;
        if (gamepad1.right_bumper)
            driveMode = 2;
    }

    // MAIN DRIVETRAIN METHOD
    private void manageDrivetrain() {
        // sets drive mode
        manageMode();

        if (driveMode == 1) {
            basicDrive();
        } else {
            planetaryDrive();
        }


        hardware.leftDrive.setPower(boost * leftPower);
        hardware.rightDrive.setPower(boost * rightPower);
        hardware.middleDrive.setPower(boost * middlePower);
    }
}
