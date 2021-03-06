package org.firstinspires.ftc.teamcode.TestOpmodes.AutoTesting.QualBot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutonomousData;
import org.firstinspires.ftc.teamcode.Hardware.QualBot_Hardware;
import org.firstinspires.ftc.teamcode.QualifierRobot.Auto;

// @Autonomous(name="Scoring in Lander", group=AutonomousData.TEST_GROUP)
public class ScoreInLander extends LinearOpMode {
    private QualBot_Hardware hardware;
    private Auto auto;

    public void runOpMode() throws InterruptedException {
        hardware = new QualBot_Hardware(hardwareMap, gamepad1, gamepad2, true);
        auto = new Auto(this, hardware);
        hardware.initHardware();

        auto.setupMineralDetector(hardwareMap);
        waitForStart();
        auto.setStartTime(System.currentTimeMillis());

        sleep(500);
        int goldPos = hardware.mineralDetector.mineralLocation();
        auto.landOnField(4);
        auto.setStartPosition(4);
        hardware.mineralDetector.disable();
        auto.sampleFromLander(goldPos, 4, false, true);

        hardware.drivetrain.faceAngle(auto.startAngle(4));
        hardware.intake.releaseMinerals(2);
        hardware.outtake.verticalSlideUp();
        auto.backupToLander(7); // Goal is to do this with the range sensor
        hardware.drivetrain.faceAngle(auto.startAngle(4));
        hardware.outtake.dump();
    }
}
