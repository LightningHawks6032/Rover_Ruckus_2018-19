package org.firstinspires.ftc.teamcode.OfficialRobot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.OfficialBot_Hardware;
import org.firstinspires.ftc.teamcode.Hardware.OfficialBot_Outtake;
import org.firstinspires.ftc.teamcode.Hardware.OfficialBot_Intake;

@TeleOp(name="Robot TeleOp", group="Iterative Opmode")
public class RobotTeleOp extends OpMode {
    private OfficialBot_Hardware hardware;

    public void init() {
        // init hardware
        hardware = new OfficialBot_Hardware(hardwareMap, gamepad1, gamepad2, false);
        hardware.initHardware();
    }

    public void loop() {
        hardware.drivetrain.manageTeleOp();
        hardware.intake.manageTeleOp();
        hardware.outtake.manageTeleOp();
        manageMarker();
        debug();
    }

    private void debug() {
        telemetry.addData("Left Motor Pow", hardware.drivetrain.getLeftPow());
        telemetry.addData("Left Encoder Val", hardware.drivetrain.getLeftEncoder().getEncoderCount());
        telemetry.addData("Right Motor Pow", hardware.drivetrain.getRightPow());
        telemetry.addData("Right Encoder Val", hardware.drivetrain.getRightEncoder().getEncoderCount());
        telemetry.addData("Horizontal Slide Encoder", hardware.intake.slideEncoder.getEncoderCount());
        telemetry.update();
    }

    public void manageMarker() {
        if (gamepad2.left_bumper)
            hardware.markerArm.setPosition(hardware.MARKER_ARM_UP);
        else if (gamepad2.right_bumper)
            hardware.markerArm.setPosition(hardware.MARKER_ARM_DOWN);
    }
}
