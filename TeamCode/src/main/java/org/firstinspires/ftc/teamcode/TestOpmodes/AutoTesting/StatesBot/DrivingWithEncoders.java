package org.firstinspires.ftc.teamcode.TestOpmodes.AutoTesting.StatesBot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutonomousData;
import org.firstinspires.ftc.teamcode.Hardware.StatesBot_Hardware;

@Autonomous(name="Encoder Test", group= AutonomousData.TEST_GROUP)
public class DrivingWithEncoders extends LinearOpMode {
    private StatesBot_Hardware hardware;

    public void runOpMode() throws InterruptedException {
        hardware = new StatesBot_Hardware(hardwareMap, gamepad1, gamepad2, false);
        hardware.initHardware();
        hardware.drivetrain.setAuto(this);
        hardware.drivetrain.setStartTime(System.currentTimeMillis());

        waitForStart();
        telemetry.addLine("Driving 10 inches at 0.5 pow");
        telemetry.update();
        hardware.drivetrain.driveDistance(1, 10, 0.5);
        telemetry.addData("DIST", hardware.drivetrain.getAverageDist());
        telemetry.addData("Left Front Encoder", hardware.drivetrain.leftFrontEncoder.linDistance());
        telemetry.addData("Right Front Encoder", hardware.drivetrain.rightFrontEncoder.linDistance());
        telemetry.addData("Left Back Encoder", hardware.drivetrain.leftBackEncoder.linDistance());
        telemetry.addData("Right Back Encoder", hardware.drivetrain.rightBackEncoder.linDistance());
        telemetry.update();
        sleep(10000);

        telemetry.addLine("Strafing 10 inches at 0.5 pow");
        telemetry.update();
        hardware.drivetrain.strafeDistance(1, 10, 0.5);
        telemetry.addData("DIST", hardware.drivetrain.getAverageDist());
        telemetry.addData("Left Front Encoder", hardware.drivetrain.leftFrontEncoder.linDistance());
        telemetry.addData("Right Front Encoder", hardware.drivetrain.rightFrontEncoder.linDistance());
        telemetry.addData("Left Back Encoder", hardware.drivetrain.leftBackEncoder.linDistance());
        telemetry.addData("Right Back Encoder", hardware.drivetrain.rightBackEncoder.linDistance());
        telemetry.update();
        sleep(10000);

    }
}