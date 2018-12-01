package org.firstinspires.ftc.teamcode.Robot1;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.Robot1_Hardware;

@TeleOp(name="Robot1 TeleOp", group="Iterative Opmode")
public class Robot1_TeleOp extends OpMode {

    Robot1_Hardware hardware;

    //Power variables
    private double slidePower,
        winchPower,
        actuPos;

    // Max Power control variables
    private double MAX_MIDDLE_POWER = 0.8;
    private double MAX_DRIVE_POWER = 0.8;

    private int driveMode;

    public void init() {
        //Initialize hardware
        hardware = new Robot1_Hardware(hardwareMap, gamepad1, false);
        hardware.initHardware();

        //Assign variables
        slidePower = 0;
        winchPower = 0;
        driveMode = 1;
        actuPos = 0.3;
    }

    public void loop() {
        hardware.drivetrain.manageTeleOp();
        manageSlide();
        manageWinch();
        manageClaws();
        manageActuator();

        if(gamepad2.a)
            hardware.markerArm.setPosition(hardware.MARKER_ARM_UP);
        else if(gamepad2.b)
            hardware.markerArm.setPosition(0.5);
        else if(gamepad2.x)
            hardware.markerArm.setPosition(hardware.MARKER_ARM_DOWN);

        telemetry.addData("left claw position", hardware.leftClaw.getPosition());
        telemetry.addData("right claw position", hardware.rightClaw.getPosition());
        telemetry.addData("linear actuator direction", hardware.linearActuator.getDirection());
        telemetry.addData("linear actuator position", hardware.linearActuator.getPosition());
        telemetry.update();
    }

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
