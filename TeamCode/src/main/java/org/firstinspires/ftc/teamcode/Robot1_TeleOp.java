package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.Robot1_Hardware;

@TeleOp(name="Robot1 TeleOp", group="Iterative Opmode")
public class Robot1_TeleOp extends OpMode {

    Robot1_Hardware hardware;

    //Power variables
    private double leftPower,
        rightPower,
        boost,
        middlePower,
        slidePower,
        winchPower,
        actuPos;

    // Max Power control variables
    private double MAX_MIDDLE_POWER = 0.8;
    private double MAX_DRIVE_POWER = 0.8;

    private int driveMode;

    public void init() {
        //Initialize hardware
        hardware = new Robot1_Hardware(hardwareMap, gamepad1);
        hardware.initHardware();

        //Assign variables
        boost = 0.7;
        rightPower = 0;
        leftPower = 0;
        slidePower = 0;
        winchPower = 0;
        driveMode = 1;
        actuPos = 0.5;
    }

    public void loop() {
        hardware.drivetrain.manageTeleOp();
        //manageDrivetrain();
        //applyBoost();
        manageSlide();
        manageWinch();
        manageClaws();
        manageActuator();

        telemetry.addData("left claw position", hardware.leftClaw.getPosition());
        telemetry.addData("right claw position", hardware.rightClaw.getPosition());
        telemetry.addData("linear actuator direction", hardware.linearActuator.getDirection());
        telemetry.addData("linear actuator position", hardware.linearActuator.getPosition());
        telemetry.update();
    }

    /*
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
        // drive 2: electric boogaloo
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
        hardware.middleDrive.setPower(boost * middlePower * 2);
    }*/

    // horizontal slide control
    private void manageSlide() {
        // could add a multiplier value
        slidePower = -gamepad2.left_stick_y; // perhaps change to x rather than y

        hardware.fastSlideMotor.setPower(slidePower);
        hardware.slowSlideMotor.setPower(slidePower*0.25);
    }

    // vertical winch control
    private void manageWinch() {
        // could add a multiplier value
        //winchPower = -gamepad2.right_stick_y;
        // winch might not work so comment it out

        hardware.winchMotor.setPower(winchPower);
    }

    // left and right claw control
    private void manageClaws() {
        // Left claw
        if (gamepad2.left_trigger > 0.5) {
            hardware.leftClaw.setPosition(hardware.LEFT_CLAW_OPEN);
        } else {
            hardware.leftClaw.setPosition(hardware.LEFT_CLAW_CLOSE);
        }

        // Right claw
        if (gamepad2.right_trigger > 0.5) {
            hardware.rightClaw.setPosition(hardware.RIGHT_CLAW_OPEN);
        } else {
            hardware.rightClaw.setPosition(hardware.RIGHT_CLAW_CLOSE);
        }
    }

    // linear actuator control
    private void manageActuator(){
        // This works
        actuPos -= gamepad2.right_stick_y * 0.001;
        hardware.linearActuator.setPosition(actuPos);
    }
}
