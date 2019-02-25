package org.firstinspires.ftc.teamcode.StatesRobot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutonomousData;
import org.firstinspires.ftc.teamcode.FieldMapping.FieldElement;
import org.firstinspires.ftc.teamcode.FieldMapping.Vector;
import org.firstinspires.ftc.teamcode.Hardware.StatesBot_Hardware;

@Autonomous(name="Red Crater Side Sample First", group= AutonomousData.OFFICIAL_GROUP)
public class RedCraterSideSampleFirst extends LinearOpMode {
    private StatesBot_Hardware hardware;
    private Auto auto;
    private final int QUADRANT = 3;
    private final int ALLIANCE = AutonomousData.RED_ALLIANCE;

    public void runOpMode() throws InterruptedException {
        hardware = new StatesBot_Hardware(hardwareMap, gamepad1, gamepad2, true);
        auto = new Auto(this, hardware);
        hardware.initHardware();

        auto.setupMineralDetector(hardwareMap);
        waitForStart();
        auto.setStartTime(System.currentTimeMillis());

        sleep(500);
        int goldPos = hardware.mineralDetector.mineralLocation();
        hardware.mineralDetector.disable();
        auto.landOnField(QUADRANT);
        auto.setStartPosition(QUADRANT);
        telemetry.addData("Robot Pos", hardware.drivetrain.robotPos.toString());
        telemetry.update();
        auto.sampleWithSlide(goldPos, QUADRANT, false);

        auto.releaseMarkerWithSlide(QUADRANT);
        hardware.intake.retractHorizontalSlide();
        auto.backToStartingPosition(ALLIANCE);
        hardware.intake.extendHorizontalSlide(0.8);

        // Attempting a cycle
        /*hardware.intake.flipOut(true);
        hardware.intake.harvest();
        hardware.drivetrain.driveDistance(1, 6, 0.9);
        hardware.intake.stopHarvester();
        hardware.intake.flipIn(true);
        hardware.intake.retractHorizontalSlide();*/
    }
}
