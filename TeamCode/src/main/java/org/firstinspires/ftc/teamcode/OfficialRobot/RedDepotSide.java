package org.firstinspires.ftc.teamcode.OfficialRobot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutonomousData;
import org.firstinspires.ftc.teamcode.FieldMapping.FieldElement;
import org.firstinspires.ftc.teamcode.Hardware.OfficialBot_Hardware;

public class RedDepotSide extends LinearOpMode {
    private OfficialBot_Hardware hardware;
    private Auto auto;

    public void runOpMode() throws InterruptedException {
        hardware = new OfficialBot_Hardware(hardwareMap, gamepad1, gamepad2, true);
        auto = new Auto(this, hardware);
        hardware.initHardware();

        auto.setupMineralDetector(hardwareMap);
        waitForStart();
        auto.setStartTime(System.currentTimeMillis());

        sleep(500);
        int goldPos = hardware.mineralDetector.mineralLocation();
        auto.landOnField();
        auto.sampleFromLander(goldPos, 4, false, false);

        hardware.drivetrain.goTo(FieldElement.RED_DEPOT, 0.6);
        auto.releaseMarker(AutonomousData.RED_ALLIANCE);
        auto.driveToCrater(AutonomousData.RED_ALLIANCE);
    }
}
