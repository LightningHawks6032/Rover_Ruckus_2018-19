/**
 * This tests mineral sampling in the fourth quadrant (red depot side)
 */

package org.firstinspires.ftc.teamcode.AutoTesting;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FieldMapping.FieldMap;
import org.firstinspires.ftc.teamcode.Hardware.Encoder;
import org.firstinspires.ftc.teamcode.Hardware.Robot1_Hardware;
import org.firstinspires.ftc.teamcode.Vision.Detectors.GoldAlignDetector;
import org.firstinspires.ftc.teamcode.Vision.Detectors.MineralDetector;

@Autonomous(name="Mineral Sampling", group="Linear Opmode")
public class Robot1_MineralSampling extends LinearOpMode {
    // Declare hardware
    private Robot1_Hardware hardware;
    private GoldAlignDetector detector;
    private FieldMap fieldMap = new FieldMap();


    private boolean encoders = true; // Do we or do we not have encoders working?

    public void runOpMode() throws InterruptedException {
        hardware = new Robot1_Hardware(hardwareMap, gamepad1, true);
        hardware.initHardware();

        detector = new GoldAlignDetector(230, 100, true); // Create detector
        detector.setupDetector(hardwareMap, 1); // Camera Index: 0 for back camera, 1 for front camera
        hardware.drivetrain.setRobotPos(20, -20); // We aren't testing nav target detection here, so assume this position

        waitForStart();

        telemetry.addLine("Mineral Sampling");
        telemetry.update();
        int goldPos = findGold();
        //turnToGold();

        telemetry.addLine("Moving to hit mineral");
        telemetry.update();
        if (goldPos == 1)
            hardware.drivetrain.goTo(fieldMap.get(4, "Left Mineral"), 0.6);
        else if (goldPos == 2)
            hardware.drivetrain.goTo(fieldMap.get(4, "Middle Mineral"), 0.6);
        else if (goldPos == 3)
            hardware.drivetrain.goTo(fieldMap.get(4, "Right Mineral"), 0.6);
        //hardware.drivetrain.driveDistance(1, 30, 0.6);

        detector.disable();
    }

    private int findGold() {
        sleep(2000);
        if (detector.getAligned())
            return 2;

        hardware.drivetrain.turn(20, false);
        if (detector.getAligned())
            return 1;

        hardware.drivetrain.turn(40, true);
        if (detector.getAligned())
            return 3;

        return 2;

    }

    private void turnToGold(){
        double turningPower;

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
