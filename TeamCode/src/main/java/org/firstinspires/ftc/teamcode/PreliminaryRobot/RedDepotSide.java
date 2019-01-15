package org.firstinspires.ftc.teamcode.PreliminaryRobot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutonomousData;
import org.firstinspires.ftc.teamcode.FieldMapping.FieldElement;
import org.firstinspires.ftc.teamcode.FieldMapping.FieldMap;
import org.firstinspires.ftc.teamcode.Hardware.PrelimBot_Hardware;

// @Autonomous(name="Robot 1 Red Depot Side", group=AutonomousData.OFFICIAL_GROUP)
public class RedDepotSide extends LinearOpMode {
    private PrelimBot_Hardware hardware;
    private Auto auto;
    private FieldMap fieldMap = new FieldMap();

    public void runOpMode() throws InterruptedException {
        // Setup auto
        hardware = new PrelimBot_Hardware(hardwareMap, gamepad1, true);
        hardware.initHardware();
        auto = new Auto(this, hardware);

        auto.setupMineralDetector(hardwareMap);
        waitForStart();
        auto.setStartTime(System.currentTimeMillis());

        hardware.drivetrain.driveDistance(1, 13, 0.5);
        auto.setStartPosition(4);

        // Sample minerals
        telemetry.addLine("Sampling Minerals");
        telemetry.update();
        auto.performMineralSampling(4, false, false);
        hardware.mineralDetector.disable();

        // Go to depot
        telemetry.addLine("Going to Depot");
        telemetry.update();
        hardware.drivetrain.goTo(fieldMap.get(FieldElement.RED_DEPOT), 0.8);

        // Dropping off marker
        telemetry.addLine("Releasing Marker");
        telemetry.update();
        auto.releaseMarker(AutonomousData.RED_ALLIANCE);

        // Returning to crater
        telemetry.addLine("Driving to Crater");
        telemetry.update();
        auto.driveToCrater(AutonomousData.RED_ALLIANCE);
    }
}