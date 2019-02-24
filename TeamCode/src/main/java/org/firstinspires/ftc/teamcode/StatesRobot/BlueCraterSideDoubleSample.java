package org.firstinspires.ftc.teamcode.StatesRobot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutonomousData;
import org.firstinspires.ftc.teamcode.FieldMapping.FieldElement;
import org.firstinspires.ftc.teamcode.FieldMapping.Vector;
import org.firstinspires.ftc.teamcode.Hardware.StatesBot_Hardware;

@Autonomous(name="Blue Crater Side Double Sample", group= AutonomousData.OFFICIAL_GROUP)
public class BlueCraterSideDoubleSample extends LinearOpMode {
    private StatesBot_Hardware hardware;
    private Auto auto;
    private final int QUADRANT = 1;
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

        auto.sampleWithSlide(goldPos, QUADRANT, false);
        auto.releaseMarkerWithSlide(QUADRANT);
        auto.doubleSampleWithSlide(goldPos, QUADRANT + 1, false);
        //hardware.drivetrain.goToBackwards(new Vector(0.8*AutonomousData.FIELD_MAP.SQUARE_LENGTH, 0.8*AutonomousData.FIELD_MAP.SQUARE_LENGTH), 0.7); // 0.6, 0.8 worked
    }
}
