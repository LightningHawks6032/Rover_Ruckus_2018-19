package org.firstinspires.ftc.teamcode.Robot1;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware.Robot1_Hardware;
import org.firstinspires.ftc.teamcode.Vision.Detectors.GoldAlignDetector;
import org.firstinspires.ftc.teamcode.Vision.Detectors.NavTargetDetector;

public class Robot1_Auto {
    private Robot1_Hardware hardware;
    private GoldAlignDetector mineralDetector;
    private NavTargetDetector navTargetDetector;

    // Constructor instantiates hardware and setups mineral detector
    public Robot1_Auto(Robot1_Hardware hardware) {
        this.hardware = hardware;
        mineralDetector = hardware.mineralDetector;
        navTargetDetector = hardware.navTargetDetector;
    }

    public void setupMineralDetector(HardwareMap hwMap) {
        mineralDetector.setupDetector(hwMap, 1);
    }

    public void setupNavigationDetector(HardwareMap hwMap) {
        navTargetDetector.setupTracker();
    }

    // Updates Robot Position and Angle with Navigation Targets
    public void updateWithNavTarget() {
        double startTime = System.currentTimeMillis();

        while (!hardware.navTargetDetector.isTargetVisible() || System.currentTimeMillis() - startTime < 3) {
            hardware.navTargetDetector.lookForTargets();
        }
        hardware.drivetrain.setRobotPos(hardware.navTargetDetector.getRobotPosition().getX(), hardware.navTargetDetector.getRobotPosition().getY());
        hardware.drivetrain.setRobotAngle((int) hardware.navTargetDetector.getRobotRotation());
    }

    public void releaseMarker() throws InterruptedException {
        hardware.markerArm.setPosition(hardware.MARKER_ARM_DOWN);
        Thread.sleep(200);
        hardware.markerArm.setPosition(hardware.MARKER_ARM_UP);
    }

    // Turn until gold mineral is aligned with robot center (NO LONGER USED)
    public void turnToGold() throws InterruptedException {
        double turningPower;

        Thread.sleep(2000);
        if (mineralDetector.isFound()) {
            double startX = mineralDetector.getXPosition();

            while (!mineralDetector.getAligned()) {
                //turn towards gold (robot center x is less than x position = turn right)
                turningPower = Math.abs(mineralDetector.getXPosition() - mineralDetector.getRobotCenterX()) / (startX - mineralDetector.getRobotCenterX()) * 0.2 + 0.1;

                hardware.drivetrain.setPowers(turningPower, -turningPower*0.5, 0);
            }
            hardware.drivetrain.setPowers(0, 0, 0);

        } else {
            //do something to get it into vision
        }
    }

}
