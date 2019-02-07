package org.firstinspires.ftc.teamcode.QualifierRobot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutonomousData;
import org.firstinspires.ftc.teamcode.FieldMapping.FieldElement;
import org.firstinspires.ftc.teamcode.Hardware.QualBot_Hardware;

@Autonomous(name="Red Depot Side", group=AutonomousData.OFFICIAL_GROUP)
public class RedDepotSide extends LinearOpMode {
    private QualBot_Hardware hardware;
    private Auto auto;
    private final int QUADRANT = 4;
    private final int ALLIANCE = AutonomousData.RED_ALLIANCE;

    public void runOpMode() throws InterruptedException {
        hardware = new QualBot_Hardware(hardwareMap, gamepad1, gamepad2, true);
        auto = new Auto(this, hardware);
        hardware.initHardware();

        auto.setupMineralDetector(hardwareMap);
        waitForStart();
        auto.setStartTime(System.currentTimeMillis());

        sleep(500);
        int goldPos = hardware.mineralDetector.mineralLocation();
        auto.landOnField(QUADRANT);
        auto.setStartPosition(QUADRANT);
        hardware.mineralDetector.disable();

        auto.sampleFromLander(goldPos, QUADRANT, false, false);
        hardware.drivetrain.goTo(FieldElement.RED_DEPOT, 0.8);
        auto.releaseMarker(ALLIANCE);

        auto.driveToCrater(ALLIANCE);
    }
}
