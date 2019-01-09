package org.firstinspires.ftc.teamcode.Robot1;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutonomousData;
import org.firstinspires.ftc.teamcode.FieldMapping.FieldElement;
import org.firstinspires.ftc.teamcode.FieldMapping.FieldMap;
import org.firstinspires.ftc.teamcode.Hardware.Robot1_Hardware;

// @Autonomous(name="Robot 1 Blue Crater Side", group=AutonomousData.OFFICIAL_GROUP)
public class Robot1_BlueCraterSide extends LinearOpMode {
    private Robot1_Hardware hardware;
    private Robot1_Auto auto;
    private FieldMap fieldMap = new FieldMap();

    public void runOpMode() throws InterruptedException {
        // Setup auto
        hardware = new Robot1_Hardware(hardwareMap, gamepad1, true);
        hardware.initHardware();
        auto = new Robot1_Auto(this, hardware);

        auto.setupMineralDetector(hardwareMap);
        waitForStart();
        auto.setStartTime(System.currentTimeMillis());

        hardware.drivetrain.driveDistance(1, 13, 0.5);
        auto.setStartPosition(1);

        // Sample minerals
        telemetry.addLine("Sampling Minerals");
        telemetry.update();
        auto.performMineralSampling(1, false, true);
        hardware.mineralDetector.disable();

        // Go to navigation target
        auto.setupNavigationDetector(hardwareMap);
        hardware.drivetrain.goTo(fieldMap.get(FieldElement.FRONT_OF_BLUE_ROVER), 0.8);
        hardware.drivetrain.face(fieldMap.get(FieldElement.BLUE_ROVER));
        hardware.drivetrain.driveDistance(1, 6, 0.5);
        auto.updateWithNavTarget();

        // Go to depot
        hardware.drivetrain.faceAngle(180);
        hardware.drivetrain.goTo(fieldMap.get(FieldElement.RED_DEPOT), 0.8);

        // Dropping off marker
        telemetry.addLine("Releasing Marker");
        telemetry.update();
        auto.releaseMarker(AutonomousData.BLUE_ALLIANCE);

        telemetry.addLine("Driving to Crater");
        telemetry.update();
        auto.driveToCrater(AutonomousData.BLUE_ALLIANCE);
    }
}
