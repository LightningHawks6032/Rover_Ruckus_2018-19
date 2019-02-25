package org.firstinspires.ftc.teamcode.StatesRobot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutonomousData;
import org.firstinspires.ftc.teamcode.FieldMapping.FieldElement;
import org.firstinspires.ftc.teamcode.FieldMapping.Vector;
import org.firstinspires.ftc.teamcode.Hardware.StatesBot_Hardware;

@Autonomous(name="Red Crater Side Marker First", group= AutonomousData.OFFICIAL_GROUP)
public class RedCraterSideMarkerFirst extends LinearOpMode {
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

        auto.releaseMarkerWithSlide(QUADRANT);
        hardware.intake.retractHorizontalSlide();
        // goes to starting loc but slightly closer to silver cargo side
        hardware.drivetrain.goToBackwards(new Vector(-0.8*AutonomousData.FIELD_MAP.SQUARE_LENGTH, -0.8*AutonomousData.FIELD_MAP.SQUARE_LENGTH), 0.7); // 0.6, 0.8 worked


        auto.sampleWithSlide(goldPos, QUADRANT, true);
        auto.scoreInLander(QUADRANT);
    }
}
