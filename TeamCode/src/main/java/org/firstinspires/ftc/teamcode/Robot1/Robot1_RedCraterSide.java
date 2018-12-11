package org.firstinspires.ftc.teamcode.Robot1;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FieldMapping.FieldElement;
import org.firstinspires.ftc.teamcode.FieldMapping.FieldMap;
import org.firstinspires.ftc.teamcode.Hardware.Robot1_Hardware;

public class Robot1_RedCraterSide extends LinearOpMode {
    private Robot1_Hardware hardware;
    private Robot1_Auto auto;
    private FieldMap fieldMap = new FieldMap();

    public void runOpMode() throws InterruptedException {
        // Setup auto
        hardware = new Robot1_Hardware(hardwareMap, gamepad1, true);
        hardware.initHardware();
        auto = new Robot1_Auto(hardware);
        waitForStart();

        // Sample code for how auto should look like
        hardware.drivetrain.face(fieldMap.get(FieldElement.BACK_SPACE));
        auto.updateWithNavTarget();
        hardware.drivetrain.face(fieldMap.get(FieldElement.RED_CRATER_MIDDLE_MINERAL));
        //auto.performMineralSampling();
        // Back up
        hardware.drivetrain.face(fieldMap.get(FieldElement.RED_FOOTPRINT));
        auto.updateWithNavTarget();
        hardware.drivetrain.goTo(fieldMap.get(FieldElement.FRONT_OF_RED_FOOTPRINT), 0.6);
        hardware.drivetrain.goTo(fieldMap.get(FieldElement.RED_DEPOT), 0.6);
        hardware.drivetrain.face(fieldMap.get(FieldElement.RED_CRATER_LEFT_EDGE));
        auto.releaseMarker();
        /* if completing mineral sampling for partner
        hardware.drivetrain.face(fieldMap.get(FieldElement.RED_DEPOT_MIDDLE_MINERAL));
        auto.performMineralSampling();
        back up
         */
        hardware.drivetrain.goTo(fieldMap.get(FieldElement.RED_CRATER_LEFT_EDGE), 0.6);
    }
}