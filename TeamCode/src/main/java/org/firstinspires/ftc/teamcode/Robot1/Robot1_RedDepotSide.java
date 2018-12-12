package org.firstinspires.ftc.teamcode.Robot1;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FieldMapping.FieldElement;
import org.firstinspires.ftc.teamcode.FieldMapping.FieldMap;
import org.firstinspires.ftc.teamcode.Hardware.Robot1_Hardware;

@Autonomous(name="Robot 1 Red Depot Side", group="Linear Opmode")
public class Robot1_RedDepotSide extends LinearOpMode {
    private Robot1_Hardware hardware;
    private Robot1_Auto auto;
    private FieldMap fieldMap = new FieldMap();

    public void runOpMode() throws InterruptedException {
        // Setup auto
        hardware = new Robot1_Hardware(hardwareMap, gamepad1, true);
        hardware.initHardware();
        auto = new Robot1_Auto(hardware);

        waitForStart();
        auto.setStartPosition(4);

        // Look to navigation targets for position
        //auto.setupNavigationDetector(hardwareMap);
        //hardware.drivetrain.face(fieldMap.get(FieldElement.RED_FOOTPRINT));
        //auto.updateWithNavTarget();

        // Face minerals
        //hardware.drivetrain.face(fieldMap.get(FieldElement.RED_DEPOT_MIDDLE_MINERAL));

        // Sample minerals
        telemetry.addLine("Sampling Minerals");
        telemetry.update();
        auto.setupMineralDetector(hardwareMap);
        auto.performMineralSampling(4, false, false);
        hardware.mineralDetector.disable();

        // Go to depot
        telemetry.addLine("Going to Depot");
        telemetry.update();
        hardware.drivetrain.goTo(fieldMap.get(FieldElement.RED_DEPOT), 0.6);

        // Dropping off marker
        telemetry.addLine("Releasing Marker");
        telemetry.update();
        auto.releaseMarker("red");

        // Returning to crater
        telemetry.addLine("Driving to Crater");
        telemetry.update();
        hardware.drivetrain.goTo(fieldMap.get(FieldElement.RED_CRATER_LEFT_EDGE), 0.6);
    }
}