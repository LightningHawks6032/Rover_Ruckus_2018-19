package org.firstinspires.ftc.teamcode.Vision.DetectorTests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.teamcode.Hardware.Robot1_Hardware;
import org.firstinspires.ftc.teamcode.Vision.DetectorTests.MineralDetectionTest;

// @TeleOp(name="Mineral Detection Tele-Op", group="DogeCV")
public class MineralDetectionDrive extends MineralDetectionTest {
    Robot1_Hardware hardware;

    public void init() {

        hardware = new Robot1_Hardware(hardwareMap, gamepad1, false);
        hardware.initHardware();
        super.init();
    }

    public void loop() {
        super.loop();

        hardware.drivetrain.manageTeleOp();
    }
}
