package org.firstinspires.ftc.teamcode.AutoTesting;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Encoder;
import org.firstinspires.ftc.teamcode.Hardware.Robot1_Hardware;
import org.firstinspires.ftc.teamcode.Vision.Detectors.GoldAlignDetector;
import org.firstinspires.ftc.teamcode.Vision.Detectors.MineralDetector;

@Autonomous(name="Mineral Sampling", group="Linear Opmode")
public class Robot1_MineralSampling extends LinearOpMode {

    // Detector object
    private GoldAlignDetector detector;

    private double turningPower;


    // Set up detector


    // Declare hardware and encoders
    private Robot1_Hardware hardware;

    private boolean encoders = true; // Do we or do we not have encoders working?

    public void runOpMode() throws InterruptedException {
        hardware = new Robot1_Hardware(hardwareMap, gamepad1);
        hardware.initHardware();

        detector = new GoldAlignDetector(230, 100, true); // Create detector
        detector.setupDetector(hardwareMap, 1); // Camera Index: 0 for back camera, 1 for front camera

        waitForStart();

        /*telemetry.addLine("Driving Forward");
        telemetry.update();
        if (encoders) {
            hardware.drivetrain.driveDistance(1, 5, 0.6);
        } else {
            hardware.drivetrain.driveForTime(0.6, 2);
        }*/

        telemetry.addLine("Mineral Sampling");
        telemetry.update();
        turnToGold();

        telemetry.addLine("Moving to hit mineral");
        telemetry.update();
        hardware.drivetrain.driveDistance(1, 30, 0.6);


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
