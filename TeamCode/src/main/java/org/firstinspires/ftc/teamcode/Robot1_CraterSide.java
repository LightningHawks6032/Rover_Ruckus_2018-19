package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Robot1_Hardware;
import org.firstinspires.ftc.teamcode.Vision.Detectors.GoldAlignDetector;

@Autonomous(name="Robot 1 Crater Side", group="Linear Opmode")
public class Robot1_CraterSide extends LinearOpMode {
    private Robot1_Hardware hardware;

    private GoldAlignDetector detector;

    private double turningPower;

    public void runOpMode() {
        hardware = new Robot1_Hardware(hardwareMap, gamepad1);
        hardware.initHardware();

        detector = new GoldAlignDetector(230, 100, true); // Create detector
        detector.setupDetector(hardwareMap, 1); // Camera Index: 0 for back camera, 1 for front camera

        waitForStart();

        telemetry.addLine("Turn to gold");
        telemetry.update();
        turnToGold();

        telemetry.addLine("Moving to hit mineral and go to crater");
        telemetry.update();
        hardware.drivetrain.driveDistance(1, 40, 0.6);
        telemetry.update();

    }

    private void turnToGold(){
        sleep(2000);
        if(detector.isFound()) {
            double startX = detector.getXPosition();

            while (!detector.getAligned()) { // robot center x is less than x position = turn right
                //turn towards gold
                turningPower = Math.abs(detector.getXPosition() - detector.getRobotCenterX()) / (startX - detector.getRobotCenterX()) * 0.4 + 0.1;

                hardware.drivetrain.setPowers(turningPower, -turningPower*0.5, 0);
            }
            turningPower = 0;
        } else {
            //do something to get it into vision
        }
    }
}
