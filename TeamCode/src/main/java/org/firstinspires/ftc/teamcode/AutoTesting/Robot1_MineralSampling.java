package org.firstinspires.ftc.teamcode.AutoTesting;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Encoder;
import org.firstinspires.ftc.teamcode.Hardware.Robot1_Hardware;

@Autonomous(name="Mineral Sampling", group="Linear Opmode")
public class Robot1_MineralSampling extends LinearOpMode {
    // Declare hardware and encoders
    Robot1_Hardware hardware;
    Encoder leftDriveEncoder;
    Encoder rightDriveEncoder;

    public void runOpMode() {
        hardware = new Robot1_Hardware(hardwareMap);
        hardware.initHardware();
        leftDriveEncoder = new Encoder(hardware.leftDrive, "Neverest", 4);
        rightDriveEncoder = new Encoder(hardware.leftDrive, "Neverest", 4);

        telemetry.addLine("Driving Forward");
        telemetry.update();
        driveDistance(1, 0.4, 5);

    }

    // @param direction: 1 is right, -1 is left
    // @param distance: distance we want robot to travel (inches)
    public void driveDistance(int direction, double power, int distance) {
        leftDriveEncoder.reset();
        rightDriveEncoder.reset();

        // Set mode to RUN_TO_POSITION
        leftDriveEncoder.runToPosition();
        rightDriveEncoder.runToPosition();

        // Set target position for encoders to run to
        leftDriveEncoder.setTarget(distance);
        rightDriveEncoder.setTarget(distance);

        // Apply power to motors
        hardware.leftDrive.setPower(direction * power);
        hardware.rightDrive.setPower(direction * power);

        while (hardware.leftDrive.isBusy() && hardware.rightDrive.isBusy()) {
            // Wait - MOTORS ARE BUSY
        }

        // Reset power
        hardware.leftDrive.setPower(0);
        hardware.rightDrive.setPower(0);
    }
}
