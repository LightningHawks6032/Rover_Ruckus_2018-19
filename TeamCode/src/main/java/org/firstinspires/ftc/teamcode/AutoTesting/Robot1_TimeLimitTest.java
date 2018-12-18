package org.firstinspires.ftc.teamcode.AutoTesting;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Robot1_Hardware;

public class Robot1_TimeLimitTest extends LinearOpMode {

    private Robot1_Hardware hardware;
    private long startTime = System.currentTimeMillis();

    @Override
    public void runOpMode() {
        hardware = new Robot1_Hardware(hardwareMap, gamepad1, false);
        hardware.initHardware();

        hardware.drivetrain.driveDistance(1, 1000, 0.6);
    }

    public void driveDistance(int direction, double distance, double pow) {
        hardware.drivetrain.leftEncoder.reset();
        hardware.drivetrain.rightEncoder.reset();

        hardware.drivetrain.leftEncoder.runToPosition();
        hardware.drivetrain.rightEncoder.runToPosition();

        hardware.drivetrain.leftEncoder.setTarget(direction * distance);
        hardware.drivetrain.rightEncoder.setTarget(direction * distance);

        hardware.drivetrain.setPowers(direction * pow, direction * pow, 0);

        while (hardware.drivetrain.leftMotor.isBusy() && hardware.drivetrain.rightMotor.isBusy() && System.currentTimeMillis() - startTime < 30000 && !isStopRequested()) {
            // WAIT - Motors are busy
        }

        hardware.drivetrain.setPowers(0, 0, 0);

        hardware.drivetrain.leftEncoder.runWithout();
        hardware.drivetrain.rightEncoder.runWithout();
    }
}
