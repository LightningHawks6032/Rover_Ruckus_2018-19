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
    private double slidePower, hangPow, actuPos;

    public void init() {
        //Initialize hardware
        hardware = new Robot1_Hardware(hardwareMap, gamepad1, false);
        hardware.initHardware();

        //Assign variables
        slidePower = 0;
        hangPow = 0;
        actuPos = 0.3;

        // Run encoders on horizontal slide motors
        hardware.slideEncoder.setup();
    }

    public void loop() {
        hardware.drivetrain.manageTeleOp();
        manageSlide();
        manageClaws();
        manageActuator();
        manageMarker();
        manageLatch();

        debug();
    }

    private void debug() {
        telemetry.addData("left claw position", hardware.leftClaw.getPosition());
        telemetry.addData("right claw position", hardware.rightClaw.getPosition());
        telemetry.addData("linear actuator direction", hardware.linearActuator.getDirection());
        telemetry.addData("linear actuator position", hardware.linearActuator.getPosition());
        telemetry.update();
    }

    private void manageMarker() {
        if(gamepad2.a)
            hardware.markerArm.setPosition(hardware.MARKER_ARM_DOWN);
        else if(gamepad2.y)
            hardware.markerArm.setPosition(hardware.MARKER_ARM_UP);
        else if(gamepad2.x)
            hardware.markerArm.setPosition(hardware.MARKER_ARM_MIDDLE);
    }

    // horizontal slide control
    private void manageSlide() {
        // could add a multiplier value
        slidePower = -gamepad2.left_stick_y; // perhaps change to x rather than y

        hardware.slideMotor.setPower(slidePower);
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
    private void manageActuator() {
        // This works
        if (gamepad2.dpad_down)
            actuPos -= 0.001;
        else if (gamepad2.dpad_up)
            actuPos += 0.001;

        // Put limit on linear actuator position
        if (actuPos > 1)
            actuPos = 1;
        if (actuPos < 0)
            actuPos = 0;

        hardware.linearActuator.setPosition(actuPos);
    }

    // lander latch control
    private void manageLatch() {
        hangPow = -gamepad2.right_stick_y;
        if (gamepad2.right_stick_y < 0)
            hardware.hangVex.setPower(hangPow * 0.3);
        else
            hardware.hangVex.setPower(hangPow * 0.1);
        hardware.hangNvst.setPower(-hangPow);
    }
}
