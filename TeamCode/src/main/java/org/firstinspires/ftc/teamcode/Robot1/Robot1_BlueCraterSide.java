package org.firstinspires.ftc.teamcode.Robot1;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FieldMapping.FieldElement;
import org.firstinspires.ftc.teamcode.FieldMapping.FieldMap;
import org.firstinspires.ftc.teamcode.Hardware.Robot1_Hardware;

public class Robot1_BlueCraterSide extends LinearOpMode {
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
        auto.setStartPosition(1);

        // Look to navigation targets for position
        hardware.drivetrain.face(fieldMap.get(FieldElement.BACK_SPACE));
        auto.updateWithNavTarget();
        hardware.drivetrain.face(fieldMap.get(FieldElement.BLUE_CRATER_MIDDLE_MINERAL));

        // Sample minerals
        auto.setupMineralDetector(hardwareMap);
        auto.performMineralSampling(1, false, true);
        hardware.mineralDetector.disable();

        // Go to navigation target
        hardware.drivetrain.goTo(fieldMap.get(FieldElement.FRONT_OF_BACK_SPACE), 0.6);
        auto.updateWithNavTarget();

        // Go to depot
        hardware.drivetrain.goTo(fieldMap.get(FieldElement.BLUE_DEPOT), 0.6);

        // Dropping off marker
        auto.releaseMarker("blue");

        // If completing mineral sampling for partner, sample minerals
        hardware.drivetrain.face(fieldMap.get(FieldElement.BLUE_DEPOT_MIDDLE_MINERAL));
        auto.performMineralSampling(2, true, true);

        // Returning to crater
        hardware.drivetrain.goTo(fieldMap.get(FieldElement.BLUE_CRATER_LEFT_EDGE), 0.6);
    }
}
