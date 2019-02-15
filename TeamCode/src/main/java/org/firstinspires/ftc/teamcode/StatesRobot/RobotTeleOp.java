package org.firstinspires.ftc.teamcode.StatesRobot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.QualBot_Hardware;
import org.firstinspires.ftc.teamcode.Hardware.StatesBot_Hardware;

@TeleOp(name="States Robot TeleOp", group="Iterative Opmode")
public class RobotTeleOp extends OpMode {
    private StatesBot_Hardware hardware;
    private boolean slideLimitingOn = true;

    public void init() {
        // init hardware
        hardware = new StatesBot_Hardware(hardwareMap, gamepad1, gamepad2, false);
        hardware.initHardware();
    }

    public void loop() {
        hardware.drivetrain.manageTeleOp();
        hardware.intake.manageTeleOp(slideLimitingOn);
        hardware.outtake.manageTeleOp(slideLimitingOn);
        manageSlideLimiting();
        debug();
    }

    private void debug() {
        telemetry.addData("Left Front Dist", hardware.drivetrain.leftFrontEncoder.linDistance());
        telemetry.addData("Right Front Dist", hardware.drivetrain.rightFrontEncoder.linDistance());
        telemetry.addData("Left Back Dist", hardware.drivetrain.leftBackEncoder.linDistance());
        telemetry.addData("Right Back Dist", hardware.drivetrain.rightBackEncoder.linDistance());
        telemetry.update();
    }

    private void manageSlideLimiting() {
        if (gamepad1.dpad_down && gamepad1.x)
            slideLimitingOn = false;
    }
}
