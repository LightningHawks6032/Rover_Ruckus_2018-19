package org.firstinspires.ftc.teamcode.TestOpmodes.AutoTesting.OfficialRobot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutonomousData;
import org.firstinspires.ftc.teamcode.Hardware.OfficialBot_Hardware;

@Autonomous(name="Encoder and Gyro Test", group= AutonomousData.TEST_GROUP)
public class EncoderGyroTest extends LinearOpMode {
    private OfficialBot_Hardware hardware;

    public void runOpMode() throws InterruptedException {
        hardware = new OfficialBot_Hardware(hardwareMap, gamepad1, gamepad2, true);
        hardware.initHardware();
        hardware.drivetrain.setAuto(this);
        hardware.drivetrain.setStartTime(System.currentTimeMillis());

        waitForStart();
        hardware.drivetrain.driveDistance(1, 10, 0.5);
        telemetry.addData("Left Encoder", hardware.drivetrain.getLeftEncoder().getEncoderCount());
        telemetry.addData("Left Distance", hardware.drivetrain.getLeftEncoder().linDistance());
        telemetry.addData("Right Encoder", hardware.drivetrain.getRightEncoder().getEncoderCount());
        telemetry.addData("Right Distance", hardware.drivetrain.getLeftEncoder().linDistance());
        telemetry.update();

        sleep(15000);
        /*
        telemetry.addLine("Turning right 90deg");
        telemetry.update();
        hardware.drivetrain.turn(90, true);

        telemetry.addLine("Turning left 90deg");
        telemetry.update();
        hardware.drivetrain.turn(90, false);
        */
    }
}