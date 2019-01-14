package org.firstinspires.ftc.teamcode.OfficialRobot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.AutonomousData;
import org.firstinspires.ftc.teamcode.FieldMapping.FieldElement;
import org.firstinspires.ftc.teamcode.FieldMapping.FieldMap;
import org.firstinspires.ftc.teamcode.FieldMapping.Vector;
import org.firstinspires.ftc.teamcode.Hardware.OfficialBot_Hardware;
import org.firstinspires.ftc.teamcode.Vision.Detectors.GoldAlignDetector;
import org.firstinspires.ftc.teamcode.Vision.Detectors.NavTargetDetector;

public class Auto {
    private LinearOpMode autonomous;
    private OfficialBot_Hardware hardware;
    private FieldMap fieldMap = new FieldMap();
    private GoldAlignDetector mineralDetector;
    private NavTargetDetector navTargetDetector;
    private long startTime;

    // Constructor instantiates hardware and setups mineral detector
    public Auto(LinearOpMode auto, OfficialBot_Hardware hardware) {
        autonomous = auto;
        this.hardware = hardware;
        mineralDetector = hardware.mineralDetector;
        navTargetDetector = hardware.navTargetDetector;

        hardware.drivetrain.setAuto(auto);
        hardware.outtake.setAuto(auto);
    }

    public void setStartTime(long time) {
        startTime = time;
        hardware.drivetrain.setStartTime(time);
        hardware.outtake.setStartTime(time);
    }

    // Computer Vision Detector Setup
    public void setupMineralDetector(HardwareMap hwMap) {
        mineralDetector.setupDetector(hwMap, 1);
    }
    public void setupNavigationDetector() {
        navTargetDetector.setupTracker();
    }

    public void setStartPosition(int quadrant) {
        switch (quadrant) {
            case 1:
                hardware.drivetrain.setRobotPos(new Vector(fieldMap.SQUARE_LENGTH, fieldMap.SQUARE_LENGTH));
                hardware.drivetrain.setRobotAngle(45);
                break;
            case 2:
                hardware.drivetrain.setRobotPos(new Vector(-fieldMap.SQUARE_LENGTH, fieldMap.SQUARE_LENGTH));
                hardware.drivetrain.setRobotAngle(135);
                break;
            case 3:
                hardware.drivetrain.setRobotPos(new Vector(-fieldMap.SQUARE_LENGTH, -fieldMap.SQUARE_LENGTH));
                hardware.drivetrain.setRobotAngle(225);
                break;
            case 4:
                hardware.drivetrain.setRobotPos(new Vector(fieldMap.SQUARE_LENGTH, -fieldMap.SQUARE_LENGTH));
                hardware.drivetrain.setRobotAngle(315);
                break;
        }
    }

    // Updates Robot Position and Angle with Navigation Targets
    public void updateWithNavTarget() throws InterruptedException {
        Thread.sleep(1000);
        long beginningTime = System.currentTimeMillis();

        // Wait for 3 seconds or until found
        while (!navTargetDetector.isTargetVisible() && System.currentTimeMillis() - beginningTime < 3000 && autoRunning()) {
            hardware.navTargetDetector.lookForTargets();
            // Swerve?
        }

        if (navTargetDetector.isTargetVisible()) {
            // Update Positional Data
            hardware.drivetrain.setRobotPos(hardware.navTargetDetector.getRobotPosition());
            hardware.drivetrain.setRobotAngle((int) Math.round(hardware.navTargetDetector.getRobotRotation()));
            hardware.drivetrain.gyroSensor.zero();

            // Debug
            //autonomous.telemetry.addData("Robot Pos", hardware.drivetrain.robotPos.toString());
            autonomous.telemetry.addData("Robot Angle From Nav", hardware.drivetrain.robotAngle);
            autonomous.telemetry.update();
        }
    }


    public void landOnField() throws InterruptedException {
        hardware.outtake.landOnField();

        hardware.drivetrain.strafeDistance(-1, 10, 1);
        hardware.drivetrain.driveDistance(1, 5, 0.5);
        hardware.drivetrain.strafeDistance(1, 5, 0.5);
        hardware.drivetrain.driveDistance(1, 8, 0.5);
    }

    /**
     * Robot performs mineral sampling after the detecting the gold mineral from the ground.
     * @param quadrant : the quadrant where the mineral sampling happens (1-4)
     * @param reverse : true if robot back is to lander, false if robot front is to lander
     * @param backup : true if we want robot to back up after knocking over gold mineral
     * @throws InterruptedException
     */
    public void sampleFromGround(int quadrant, boolean reverse, boolean backup) throws InterruptedException {
        int goldPos = 2; // by default

        // Generates minerals to choose from
        FieldElement[] minerals = samplingField(quadrant, reverse);

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

        // Back up if necessary
        if (backup) {
            if (!reverse)
                hardware.drivetrain.driveDistance(-1, hardware.drivetrain.robotPos.distanceFrom(startPos) * 3/4, 0.6);
            else
                hardware.drivetrain.driveDistance(-1, hardware.drivetrain.robotPos.distanceFrom(startPos), 0.6);
            hardware.drivetrain.updatePosFromEncoders();
            hardware.drivetrain.setRobotAngle((360 + hardware.drivetrain.robotAngle - hardware.drivetrain.gyroSensor.getAngle()) % 360);
        }
    }

    /**
     * Robot performs mineral sampling after the detecting the gold mineral while on the lander.
     * @param goldPos : the position of the gold mineral determined while the robot is hanging at beginning of autonomous
     * @param quadrant : the quadrant where the mineral sampling happens (1-4)
     * @param reverse : true if robot back is to lander, false if robot front is to lander
     * @param backup : true if we want robot to back up after knocking over gold mineral
     * @throws InterruptedException
     */
    public void sampleFromLander(int goldPos, int quadrant, boolean reverse, boolean backup) throws InterruptedException {
        // Generates minerals to choose from
        FieldElement[] minerals = samplingField(quadrant, reverse);

        // Go to Gold
        Vector startPos = hardware.drivetrain.robotPos;
        if (goldPos == 1)
            hardware.drivetrain.goTo(fieldMap.get(minerals[0]), 0.8);
        else if (goldPos == 2)
            hardware.drivetrain.goTo(fieldMap.get(minerals[1]), 0.8);
        else if (goldPos == 3) {
            hardware.drivetrain.goTo(fieldMap.get(minerals[2]), 0.8);
        }

        // Back up if necessary
        if (backup) {
            if (!reverse)
                hardware.drivetrain.driveDistance(-1, hardware.drivetrain.robotPos.distanceFrom(startPos) * 3/4, 0.6);
            else
                hardware.drivetrain.driveDistance(-1, hardware.drivetrain.robotPos.distanceFrom(startPos), 0.6);
            hardware.drivetrain.updatePosFromEncoders();
            hardware.drivetrain.setRobotAngle((360 + hardware.drivetrain.robotAngle - hardware.drivetrain.gyroSensor.getAngle()) % 360);
        }
    }

    /**
     * Create an array of the minerals, represented as FieldElements, in a particular sampling field.
     * @param quadrant : the quadrant where the sampling field is happens (1-4)
     * @param reverse : true if robot back is to lander, false if robot front is to lander
     * @return the array of minerals, from left to right from the POV of the robot.
     */
    private FieldElement[] samplingField(int quadrant, boolean reverse) {
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

        return minerals;
    }

    public void driveToCrater(int alliance) throws InterruptedException {
        if (alliance == AutonomousData.RED_ALLIANCE) {
            hardware.drivetrain.faceAngle(180);
        } else if (alliance == AutonomousData.BLUE_ALLIANCE) {
            hardware.drivetrain.faceAngle(0);
        }

        hardware.drivetrain.strafeForTime(-0.8, 3/2);
        hardware.drivetrain.driveDistance(1, fieldMap.SQUARE_LENGTH * 4, 1);
        //extendHorizontalSlide(0.4, 1);
    }

    // Used to break all while loops when an opmode stops
    private boolean autoRunning() {
        return System.currentTimeMillis() - startTime <= AutonomousData.TIME_LIMIT && !autonomous.isStopRequested();
    }
}
