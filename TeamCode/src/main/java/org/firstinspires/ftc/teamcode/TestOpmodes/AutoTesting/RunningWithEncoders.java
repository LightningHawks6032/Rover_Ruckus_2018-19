package org.firstinspires.ftc.teamcode.TestOpmodes.AutoTesting;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutonomousData;
import org.firstinspires.ftc.teamcode.Hardware.OfficialBot_Hardware;

// @Autonomous(name="Encoder Test", group= AutonomousData.TEST_GROUP)
public class RunningWithEncoders extends LinearOpMode {
    private OfficialBot_Hardware hardware;

    public void runOpMode() throws InterruptedException {
        hardware = new OfficialBot_Hardware(hardwareMap, gamepad1, gamepad2, false);
        hardware.initHardware();
        hardware.drivetrain.setAuto(this);
        hardware.intake.setAuto(this);
        hardware.drivetrain.setStartTime(System.currentTimeMillis());
        hardware.intake.setStartTime(System.currentTimeMillis());

        waitForStart();
        telemetry.addLine("Driving 10 inches at 0.5 pow");
        telemetry.update();
        hardware.drivetrain.driveDistance(1, 10, 1);
        telemetry.addData("Left Encoder", hardware.drivetrain.getLeftEncoder().getEncoderCount());
        telemetry.addData("Left Distance", hardware.drivetrain.getLeftEncoder().linDistance());
        telemetry.addData("Right Encoder", hardware.drivetrain.getRightEncoder().getEncoderCount());
        telemetry.addData("Right Distance", hardware.drivetrain.getLeftEncoder().linDistance());
        telemetry.update();

        sleep(5000);

        telemetry.addLine("Driving 20 inches at 0.5 pow");
        telemetry.update();
        hardware.drivetrain.driveDistance(1, 20, 1);
        telemetry.addData("Left Encoder", hardware.drivetrain.getLeftEncoder().getEncoderCount());
        telemetry.addData("Left Distance", hardware.drivetrain.getLeftEncoder().linDistance());
        telemetry.addData("Right Encoder", hardware.drivetrain.getRightEncoder().getEncoderCount());
        telemetry.addData("Right Distance", hardware.drivetrain.getLeftEncoder().linDistance());
        telemetry.update();

        sleep(5000);

    }
}