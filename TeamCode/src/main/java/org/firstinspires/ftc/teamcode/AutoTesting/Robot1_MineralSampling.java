package org.firstinspires.ftc.teamcode.AutoTesting;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Encoder;
import org.firstinspires.ftc.teamcode.Hardware.Robot1_Hardware;

@Autonomous(name="Mineral Sampling", group="Linear Opmode")
public class Robot1_MineralSampling extends LinearOpMode {
    // Declare hardware and encoders
    Robot1_Hardware hardware;

    public void runOpMode() {
        hardware = new Robot1_Hardware(hardwareMap, gamepad1);
        hardware.initHardware();


        telemetry.addLine("Driving Forward");
        telemetry.update();
        hardware.drivetrain.driveDistance(1, 5, 0.4);

    }
}
