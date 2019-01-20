package org.firstinspires.ftc.teamcode.OfficialRobot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutonomousData;
import org.firstinspires.ftc.teamcode.FieldMapping.FieldElement;
import org.firstinspires.ftc.teamcode.Hardware.OfficialBot_Hardware;

@Autonomous(name="Red Depot Side", group=AutonomousData.OFFICIAL_GROUP)
public class RedDepotSide extends LinearOpMode {
    private OfficialBot_Hardware hardware;
    private Auto auto;
    private final int QUADRANT = 4;
    private final int ALLIANCE = AutonomousData.RED_ALLIANCE;

    public void runOpMode() throws InterruptedException {
        hardware = new OfficialBot_Hardware(hardwareMap, gamepad1, gamepad2, true);
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

        //telemetry.addLine("Sampling");
        //telemetry.update();
        auto.sampleFromLander(goldPos, QUADRANT, false, false);

        //hardware.drivetrain.goTo(FieldElement.RED_DEPOT, 0.6);
        //auto.releaseMarker(ALLIANCE);

        //auto.driveToCrater(ALLIANCE);
    }
}
