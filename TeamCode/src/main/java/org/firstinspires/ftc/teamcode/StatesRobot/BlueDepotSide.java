package org.firstinspires.ftc.teamcode.StatesRobot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutonomousData;
import org.firstinspires.ftc.teamcode.FieldMapping.FieldElement;
import org.firstinspires.ftc.teamcode.Hardware.QualBot_Hardware;
import org.firstinspires.ftc.teamcode.Hardware.StatesBot_Hardware;
import org.firstinspires.ftc.teamcode.StatesRobot.Auto;

@Autonomous(name="Blue Depot Side", group= AutonomousData.OFFICIAL_GROUP)
public class BlueDepotSide extends LinearOpMode {
    private StatesBot_Hardware hardware;
    private Auto auto;
    private final int QUADRANT = 2;
    private final int ALLIANCE = AutonomousData.BLUE_ALLIANCE;

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
        auto.releaseMarkerWithSlide();
        auto.sampleWithSlide(goldPos, QUADRANT, false);
        auto.scoreInLander(QUADRANT);

        hardware.drivetrain.driveDistance(1, 6, 0.6);
        hardware.drivetrain.updatePosAfterDrive(1);
        hardware.drivetrain.goTo(AutonomousData.FIELD_MAP.get(FieldElement.FRONT_OF_BACK_SPACE), 0.6);
        hardware.drivetrain.faceAngle(-90);
        hardware.drivetrain.faceAngle(-90);
        hardware.intake.extendHorizontalSlide(0.7);

    }
}
