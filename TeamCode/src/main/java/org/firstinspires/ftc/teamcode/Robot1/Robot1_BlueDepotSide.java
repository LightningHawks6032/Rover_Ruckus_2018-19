package org.firstinspires.ftc.teamcode.Robot1;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FieldMapping.FieldElement;
import org.firstinspires.ftc.teamcode.FieldMapping.FieldMap;
import org.firstinspires.ftc.teamcode.Hardware.Robot1_Hardware;

public class Robot1_BlueDepotSide extends LinearOpMode {
    private Robot1_Hardware hardware;
    private Robot1_Auto auto;
    private FieldMap fieldMap = new FieldMap();

    public void runOpMode() throws InterruptedException {
        // Setup auto
        hardware = new Robot1_Hardware(hardwareMap, gamepad1, true);
        hardware.initHardware();
        auto = new Robot1_Auto(hardware);

        auto.setupNavigationDetector(hardwareMap);
        waitForStart();
        auto.setStartPosition(2);

        // Look to navigation targets for position
        hardware.drivetrain.face(fieldMap.get(FieldElement.BLUE_ROVER));
        auto.updateWithNavTarget();
        hardware.drivetrain.face(fieldMap.get(FieldElement.BLUE_DEPOT_MIDDLE_MINERAL));

        // Sample minerals
        telemetry.addLine("Sampling Minerals");
        telemetry.update();
        auto.setupMineralDetector(hardwareMap);
        auto.performMineralSampling(2, false, false);
        hardware.mineralDetector.disable();

        // Go to depot
        telemetry.addLine("Going to Depot");
        telemetry.update();
        hardware.drivetrain.goTo(fieldMap.get(FieldElement.BLUE_DEPOT), 0.6);

        // Dropping off marker
        telemetry.addLine("Releasing Marker");
        telemetry.update();
        auto.releaseMarker("blue");

        // Returning to crater
        telemetry.addLine("Driving to Crater");
        telemetry.update();
        hardware.drivetrain.goTo(fieldMap.get(FieldElement.BLUE_CRATER_LEFT_EDGE), 0.6);
    }
}