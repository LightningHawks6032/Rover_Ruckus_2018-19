package org.firstinspires.ftc.teamcode.Robot1;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.FieldMapping.FieldElement;
import org.firstinspires.ftc.teamcode.FieldMapping.FieldMap;
import org.firstinspires.ftc.teamcode.FieldMapping.Vector;
import org.firstinspires.ftc.teamcode.Hardware.Robot1_Hardware;
import org.firstinspires.ftc.teamcode.Vision.Detectors.GoldAlignDetector;
import org.firstinspires.ftc.teamcode.Vision.Detectors.NavTargetDetector;

import java.lang.reflect.Field;

public class Robot1_Auto {
    private Robot1_Hardware hardware;
    private FieldMap fieldMap = new FieldMap();
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

    public void setStartPosition(int quadrant) {
        switch (quadrant) {
            case 1:
                hardware.drivetrain.setRobotPos(fieldMap.SQUARE_LENGTH, fieldMap.SQUARE_LENGTH);
                hardware.drivetrain.setRobotAngle(45);
                break;
            case 2:
                hardware.drivetrain.setRobotPos(-fieldMap.SQUARE_LENGTH, fieldMap.SQUARE_LENGTH);
                hardware.drivetrain.setRobotAngle(135);
                break;
            case 3:
                hardware.drivetrain.setRobotPos(-fieldMap.SQUARE_LENGTH, -fieldMap.SQUARE_LENGTH);
                hardware.drivetrain.setRobotAngle(225);
                break;
            case 4:
                hardware.drivetrain.setRobotPos(fieldMap.SQUARE_LENGTH, -fieldMap.SQUARE_LENGTH);
                hardware.drivetrain.setRobotAngle(315);
                break;
        }
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

    /**
     * Robot performs mineral sampling by moving the gold mineral off of its starting position
     * @param quadrant : the quadrant where the mineral sampling happens (1-4)
     * @param reverse : true if robot back is to lander, false if robot front is to lander
     * @param backup : true if we want robot to back up after knocking over gold mineral
     * @throws InterruptedException
     */
    public void performMineralSampling(int quadrant, boolean reverse, boolean backup) throws InterruptedException {
        int goldPos = findGold();
        FieldElement[] minerals = new FieldElement[3];

        // Generates minerals to choose from
        switch (quadrant) {
            case 1:
                minerals[0] = !reverse ? FieldElement.BLUE_CRATER_LEFT_MINERAL : FieldElement.BLUE_CRATER_RIGHT_MINERAL;
                minerals[1] = FieldElement.BLUE_CRATER_MIDDLE_MINERAL;
                minerals[2] = reverse ? FieldElement.BLUE_CRATER_LEFT_MINERAL : FieldElement.BLUE_CRATER_RIGHT_MINERAL;
                break;
            case 2:
                minerals[0] = !reverse ? FieldElement.BLUE_DEPOT_LEFT_MINERAL : FieldElement.BLUE_DEPOT_RIGHT_MINERAL;
                minerals[1] = FieldElement.BLUE_DEPOT_MIDDLE_MINERAL;
                minerals[2] = reverse ? FieldElement.BLUE_DEPOT_LEFT_MINERAL : FieldElement.BLUE_DEPOT_RIGHT_MINERAL;
                break;
            case 3:
                minerals[0] = !reverse ? FieldElement.RED_CRATER_LEFT_MINERAL : FieldElement.RED_CRATER_RIGHT_MINERAL;
                minerals[1] = FieldElement.RED_CRATER_MIDDLE_MINERAL;
                minerals[2] = reverse ? FieldElement.RED_CRATER_LEFT_MINERAL : FieldElement.RED_CRATER_RIGHT_MINERAL;
                break;
            case 4:
                minerals[0] = !reverse ? FieldElement.RED_DEPOT_LEFT_MINERAL : FieldElement.RED_DEPOT_RIGHT_MINERAL;
                minerals[1] = FieldElement.RED_DEPOT_MIDDLE_MINERAL;
                minerals[2] = reverse ? FieldElement.RED_DEPOT_LEFT_MINERAL : FieldElement.RED_DEPOT_RIGHT_MINERAL;
                break;
        }

        Vector startPos = hardware.drivetrain.robotPos;

        if (goldPos == 1)
            hardware.drivetrain.goTo(fieldMap.get(minerals[0]), 0.6);
        else if (goldPos == 2)
            hardware.drivetrain.goTo(fieldMap.get(minerals[1]), 0.6);
        else if (goldPos == 3)
            hardware.drivetrain.goTo(fieldMap.get(minerals[2]), 0.6);

        if (backup)
            hardware.drivetrain.driveDistance(-1, hardware.drivetrain.robotPos.distanceFrom(startPos), 0.6);

    }

    public int findGold() throws InterruptedException {
        Thread.sleep(1000);
        if (mineralDetector.getAligned()) {
            return 2;
        }

        hardware.drivetrain.turn(30, false);
        Thread.sleep(500);
        if (mineralDetector.getAligned()) {
            hardware.drivetrain.turn(30, true); // Turn back
            return 1;
        }

        hardware.drivetrain.turn(60, true);
        Thread.sleep(500);
        if (mineralDetector.getAligned()) {
            hardware.drivetrain.turn(30, false); // Turn back
            return 3;
        }

        hardware.drivetrain.turn(30, false); // Turn back


        return 2;
    }


    public void releaseMarker(String alliance) throws InterruptedException {
        if (alliance.equals("red"))
            hardware.drivetrain.face(fieldMap.get(FieldElement.RED_CRATER_LEFT_EDGE));
        else if (alliance.equals("blue"))
            hardware.drivetrain.face(fieldMap.get(FieldElement.BLUE_CRATER_LEFT_EDGE));

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
