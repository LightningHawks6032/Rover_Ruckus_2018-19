package org.firstinspires.ftc.teamcode.TestOpmodes.AutoTesting.OfficialRobot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.OfficialBot_Hardware;

@Autonomous
public class TurningWithGyro extends LinearOpMode {
    private OfficialBot_Hardware hardware;

    public void runOpMode() throws InterruptedException {
        hardware = new OfficialBot_Hardware(hardwareMap, gamepad1, gamepad2, true);
        hardware.initHardware();
        hardware.drivetrain.setAuto(this);
        hardware.drivetrain.setStartTime(System.currentTimeMillis());

        waitForStart();

        telemetry.addLine("Turning right 90deg");
        telemetry.update();
        hardware.drivetrain.turn(90, true);

        telemetry.addLine("Turning left 90deg");
        telemetry.update();
        hardware.drivetrain.turn(90, false);
    }
}