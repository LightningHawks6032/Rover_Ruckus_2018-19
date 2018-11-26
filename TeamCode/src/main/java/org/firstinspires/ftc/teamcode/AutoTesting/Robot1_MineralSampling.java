package org.firstinspires.ftc.teamcode.AutoTesting;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Encoder;
import org.firstinspires.ftc.teamcode.Hardware.Robot1_Hardware;

@Autonomous(name="Mineral Sampling", group="Linear Opmode")
public class Robot1_MineralSampling extends LinearOpMode {
    // Declare hardware and encoders
    private Robot1_Hardware hardware;

    private boolean encoders = false; // Do we or do we not have encoders working?

    public void runOpMode() throws InterruptedException {
        hardware = new Robot1_Hardware(hardwareMap, gamepad1);
        hardware.initHardware();

        telemetry.addLine("Driving Forward");
        telemetry.update();
        if (encoders) {
            hardware.drivetrain.driveDistance(1, 5, 0.4);
        } else {
            hardware.drivetrain.driveForTime(0.4, 2);
        }
    }
}
