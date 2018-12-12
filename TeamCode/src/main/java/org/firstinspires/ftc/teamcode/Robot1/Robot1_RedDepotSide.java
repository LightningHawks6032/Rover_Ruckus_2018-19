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
        auto.setupNavigationDetector(hardwareMap);
        hardware.drivetrain.face(fieldMap.get(FieldElement.FRONT_CRATERS));
        auto.updateWithNavTarget();

        // Face minerals
        hardware.drivetrain.face(fieldMap.get(FieldElement.RED_DEPOT_MIDDLE_MINERAL));

        // Sample minerals
        auto.setupMineralDetector(hardwareMap);
        auto.performMineralSampling(4, false, false);
        hardware.mineralDetector.disable();

        // Go to depot
        hardware.drivetrain.goTo(fieldMap.get(FieldElement.RED_DEPOT), 0.6);

        // Dropping off marker
        auto.releaseMarker("red");

        // Returning to crater
        hardware.drivetrain.goTo(fieldMap.get(FieldElement.RED_CRATER_LEFT_EDGE), 0.6);
    }
}