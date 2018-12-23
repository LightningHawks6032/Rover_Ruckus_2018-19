package org.firstinspires.ftc.teamcode.Robot1;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FieldMapping.FieldElement;
import org.firstinspires.ftc.teamcode.FieldMapping.FieldMap;
import org.firstinspires.ftc.teamcode.Hardware.Robot1_Hardware;

@Autonomous(name="Robot 1 Blue Crater Side", group="Linear Opmode")
public class Robot1_BlueCraterSide extends LinearOpMode {
    private Robot1_Hardware hardware;
    private Robot1_Auto auto;
    private FieldMap fieldMap = new FieldMap();

    public void runOpMode() throws InterruptedException {
        long startTime = System.currentTimeMillis();

        // Setup auto
        hardware = new Robot1_Hardware(hardwareMap, gamepad1, true);
        hardware.initHardware();
        auto = new Robot1_Auto(this, hardware);

        auto.setupMineralDetector(hardwareMap);
        waitForStart();
        auto.setStartTime(System.currentTimeMillis());

        auto.landOnField();
        auto.setStartPosition(1);

        // Sample minerals
        telemetry.addLine("Sampling Minerals");
        telemetry.update();
        auto.performMineralSampling(1, false, false);
        hardware.mineralDetector.disable();

        // Driving to Crater
        telemetry.addLine("Driving to Crater");
        telemetry.update();
        hardware.drivetrain.faceAngle(45);
        hardware.drivetrain.driveDistance(1, 5, 0.5);
        auto.extendHorizontalSlide(0.3, 1);
    }
}
