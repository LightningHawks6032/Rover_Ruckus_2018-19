package org.firstinspires.ftc.teamcode.QualifierRobot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutonomousData;
import org.firstinspires.ftc.teamcode.FieldMapping.FieldElement;
import org.firstinspires.ftc.teamcode.Hardware.QualBot_Hardware;

@Autonomous(name="Blue Land and Park", group=AutonomousData.OFFICIAL_GROUP)
public class BlueLandPark extends LinearOpMode {
    private QualBot_Hardware hardware;
    private Auto auto;

    public void runOpMode() throws InterruptedException {
        hardware = new QualBot_Hardware(hardwareMap, gamepad1, gamepad2, true);
        auto = new Auto(this, hardware);
        hardware.initHardware();

        waitForStart();
        auto.setStartTime(System.currentTimeMillis());

        auto.landOnField(1);
        hardware.drivetrain.goTo(FieldElement.FRONT_OF_BACK_SPACE, 0.8);
        hardware.drivetrain.faceAngle(80);
        hardware.drivetrain.driveDistance(1, AutonomousData.FIELD_MAP.SQUARE_LENGTH, 1);
        hardware.intake.horizontalSlide.setPower(1);
        Thread.sleep(800);
        hardware.intake.horizontalSlide.setPower(0);
    }
}
