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
    private long startTime;

    final long autoTimeLimit = 300000; // Autonomous time limit in milliseconds

    public boolean stopRequested = false;

    // Constructor instantiates hardware and setups mineral detector
    public Robot1_Auto(Robot1_Hardware hardware, long startTime) {
        this.hardware = hardware;
        mineralDetector = hardware.mineralDetector;
        navTargetDetector = hardware.navTargetDetector;
        this.startTime = startTime;
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
        long startTime = System.currentTimeMillis();

        // Wait for 3 seconds or until found
        while (!navTargetDetector.isTargetVisible() || System.currentTimeMillis() - startTime < 3000) {
            hardware.navTargetDetector.lookForTargets();
        }

        if (navTargetDetector.isTargetVisible()) {
            hardware.drivetrain.setRobotPos(hardware.navTargetDetector.getRobotPosition().getX(), hardware.navTargetDetector.getRobotPosition().getY());
            hardware.drivetrain.setRobotAngle((int) hardware.navTargetDetector.getRobotRotation());
        }
    }

    public void landOnField() throws InterruptedException {
        hardware.hangEncoder.reset();
        hardware.hangEncoder.runToPosition();
        hardware.hangEncoder.setEncoderTarget(16500);
        hardware.hangNvst.setPower(-1);
        while (hardware.hangNvst.isBusy() && autoRunning()) {
            // WAIT - Motor is busy
        }
        hardware.hangNvst.setPower(0);

        hardware.drivetrain.strafeDistance(-1, 10, 1);
        hardware.drivetrain.driveDistance(1, 5, 0.5);
        hardware.drivetrain.strafeDistance(1, 5, 0.5);
        hardware.drivetrain.driveDistance(1, 8, 0.5);
    }

    /**
     * Robot performs mineral sampling by moving the gold mineral off of its starting position
     * @param quadrant : the quadrant where the mineral sampling happens (1-4)
     * @param reverse : true if robot back is to lander, false if robot front is to lander
     * @param backup : true if we want robot to back up after knocking over gold mineral
     * @throws InterruptedException
     */
    public void performMineralSampling(int quadrant, boolean reverse, boolean backup) throws InterruptedException {
        int goldPos = 2; // by default

        // Generates minerals to choose from
        FieldElement[] minerals = new FieldElement[3];
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

        // Find Gold
        boolean found = false;
        Thread.sleep(500);
        if (mineralDetector.getAligned()) { // gold is middle
            goldPos = 2;
            found = true;
        }

        if (!found) {
            hardware.drivetrain.face(fieldMap.get(minerals[0]));
            //Thread.sleep(500);
            if (mineralDetector.getAligned()) { // gold is left
                goldPos = 1;
                found = true;
            }
        }

        if (!found) {
            hardware.drivetrain.face(fieldMap.get(minerals[2]));
            //Thread.sleep(500);
            if (mineralDetector.getAligned()) { // gold is right
                goldPos = 3;
                found = true;
            }
        }

        if (!found)
            hardware.drivetrain.face(fieldMap.get(minerals[1])); // Turn back


        // Go to Gold
        Vector startPos = hardware.drivetrain.robotPos;
        if (goldPos == 1)
            hardware.drivetrain.goTo(fieldMap.get(minerals[0]), 0.8);
        else if (goldPos == 2)
            hardware.drivetrain.goTo(fieldMap.get(minerals[1]), 0.8);
        else if (goldPos == 3) {
            hardware.drivetrain.goTo(fieldMap.get(minerals[2]), 0.8);
        }

        if (backup) {
            hardware.drivetrain.driveDistance(-1, hardware.drivetrain.robotPos.distanceFrom(startPos), 0.6);
            hardware.drivetrain.setRobotPos(startPos.getX(), startPos.getY());
            hardware.drivetrain.setRobotAngle((int) ((360 + hardware.drivetrain.robotAngle - hardware.drivetrain.gyroSensor.getAngle()) % 360));
        }

    }

    private int findGold() throws InterruptedException {
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
            hardware.drivetrain.faceAngle(270);
        else if (alliance.equals("blue"))
            hardware.drivetrain.faceAngle(90);
        hardware.drivetrain.driveDistance(1, 5, 0.6);
        hardware.drivetrain.strafeForTime(0.8, 1); // used to be BEFORE the above if statements

        hardware.markerArm.setPosition(hardware.MARKER_ARM_DOWN);
        Thread.sleep(1000);
        hardware.markerArm.setPosition(hardware.MARKER_ARM_UP);
    }

    public void driveToCrater(String alliance) throws InterruptedException {
        if (alliance.equals("red")) {
            hardware.drivetrain.faceAngle(180);
        } else if (alliance.equals("blue")) {
            hardware.drivetrain.faceAngle(0);
        }

        hardware.drivetrain.strafeForTime(-0.8, 3/2);
        hardware.drivetrain.driveDistance(1, fieldMap.SQUARE_LENGTH * 4, 1);
        extendHorizontalSlide(0.4, 1);
    }

    public void extendHorizontalSlide(double power, long seconds) throws InterruptedException {
        hardware.slideMotor.setPower(power);
        Thread.sleep(seconds*1000);
        hardware.slideMotor.setPower(0);
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

    public boolean autoRunning() {
        return System.currentTimeMillis() - startTime >= autoTimeLimit && !stopRequested;
    }

}
