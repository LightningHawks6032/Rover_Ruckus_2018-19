package org.firstinspires.ftc.teamcode.OfficialRobot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
        hardware.intake.setAuto(auto);
        hardware.outtake.setAuto(auto);
        resetManipsEncoders();
    }

    public void setStartTime(long time) {
        startTime = time;
        hardware.drivetrain.setStartTime(time);
        hardware.intake.setStartTime(time);
        hardware.outtake.setStartTime(time);
    }

    // Computer Vision Detector Setup
    public void setupMineralDetector(HardwareMap hwMap) {
        mineralDetector.setupDetector(hwMap, 1);
    }
    public void setupNavigationDetector() {
        navTargetDetector.setupTracker();
    }

    private void resetManipsEncoders() {
        hardware.intake.flipEncoder.reset();
        hardware.intake.slideEncoder.reset();
        hardware.outtake.leftVertEncoder.reset();
        hardware.outtake.rightVertEncoder.reset();
    }

    // Sets up the starting position of the robot after it has landed and oriented itself on field
    public void setStartPosition(int quadrant) throws InterruptedException {
        hardware.drivetrain.faceAngle(startAngle(quadrant));

        double distFromLander = hardware.rangeSensor.getDistance(DistanceUnit.INCH);
        double coordinateOffset = (distFromLander + hardware.RANGE_SENSOR_DISPLACEMENT) / Math.sqrt(2);

        // Set lander position and robot position
        Vector landerPos;
        switch (quadrant) {
            case 1:
                landerPos = fieldMap.get(FieldElement.QUAD_1_LANDER_WALL);
                hardware.drivetrain.setRobotPos(landerPos.sum(new Vector(coordinateOffset, coordinateOffset)));
                break;
            case 2:
                landerPos = fieldMap.get(FieldElement.QUAD_2_LANDER_WALL);
                hardware.drivetrain.setRobotPos(landerPos.sum(new Vector(-coordinateOffset, coordinateOffset)));
                break;
            case 3:
                landerPos = fieldMap.get(FieldElement.QUAD_3_LANDER_WALL);
                hardware.drivetrain.setRobotPos(landerPos.sum(new Vector(-coordinateOffset, -coordinateOffset)));
                break;
            case 4:
                landerPos = fieldMap.get(FieldElement.QUAD_4_LANDER_WALL);
                hardware.drivetrain.setRobotPos(landerPos.sum(new Vector(coordinateOffset, -coordinateOffset)));
                break;
        }

        hardware.drivetrain.updateAngleFromIMU();
    }

    // Returns the starting angle of the robot dependent on its starting quadrant
    public int startAngle(int quadrant) {
        return 45 + 90 * (quadrant - 1);
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


    public void landOnField(int quadrant) throws InterruptedException {
        // Land on field
        hardware.outtake.landOnField();

        // Set up IMU readings and robot angle
        Thread.sleep(500);
        hardware.drivetrain.setInitialIMUHeading();
        hardware.drivetrain.setInitialRobotAngle(startAngle(quadrant));

        // Move away from lander
        hardware.drivetrain.strafeDistance(-1, 4, 1);
        hardware.drivetrain.driveDistance(1, 8, 0.6);
        hardware.drivetrain.strafeDistance(1, 6, 1);
        hardware.outtake.verticalSlideDown();
    }

    public void driveAndLowerSlide(int direction, double distance, double pow) throws InterruptedException {
        hardware.drivetrain.leftEncoder.reset();
        hardware.drivetrain.rightEncoder.reset();

        hardware.drivetrain.leftEncoder.runToPosition();
        hardware.drivetrain.rightEncoder.runToPosition();

        hardware.drivetrain.leftEncoder.setTarget(direction * distance);
        hardware.drivetrain.rightEncoder.setTarget(direction * distance);

        hardware.drivetrain.setPowers(direction * pow, direction * pow, 0);

        while (hardware.drivetrain.leftMotor.isBusy() && hardware.drivetrain.rightMotor.isBusy() && autoRunning()) {
            int slideEncoderVal = (hardware.outtake.leftVertEncoder.getEncoderCount() + hardware.outtake.rightVertEncoder.getEncoderCount()) / 2; // average
            if (slideEncoderVal > hardware.outtake.VERTICAL_SLIDE_MIN) {
                hardware.outtake.leftVertical.setPower(-1);
                hardware.outtake.rightVertical.setPower(-1);
            }
        }

        hardware.drivetrain.setPowers(0, 0, 0);
        hardware.drivetrain.leftEncoder.runWithout();
        hardware.drivetrain.rightEncoder.runWithout();
        hardware.drivetrain.updateAngleFromIMU();
    }

    private void goToIntake(Vector location, double pow) throws InterruptedException {
        hardware.drivetrain.face(location); // Turn to face location
        hardware.intake.flipOut();
        hardware.intake.harvest();
        driveAndLowerSlide(1, location.distanceFrom(hardware.drivetrain.robotPos), pow); // Drive to location
        hardware.outtake.verticalSlideDown(); // in case vertical slide did not reach down
        hardware.drivetrain.updatePosFromEncoders();
        hardware.drivetrain.updateAngleFromIMU();
        hardware.intake.stopHarvesting();
        hardware.intake.flipIn();
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
            if (mineralDetector.getAligned()) { // gold is left
                goldPos = 1;
                found = true;
            }
        }
        if (!found) {
            hardware.drivetrain.face(fieldMap.get(minerals[2]));
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
            hardware.drivetrain.goTo(fieldMap.get(minerals[0]), 0.4);
        else if (goldPos == 2)
            hardware.drivetrain.goTo(fieldMap.get(minerals[1]), 0.4);
        else if (goldPos == 3) {
            hardware.drivetrain.goTo(fieldMap.get(minerals[2]), 0.4);
        }

        // Back up if necessary
        if (backup) {
            if (!reverse)
                hardware.drivetrain.driveDistance(-1, hardware.drivetrain.robotPos.distanceFrom(startPos) * 3/4, 0.6);
            else
                hardware.drivetrain.driveDistance(-1, hardware.drivetrain.robotPos.distanceFrom(startPos), 0.6);
            hardware.drivetrain.updatePosFromEncoders();
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
        double pow = 0.7;

        // Go to Gold
        Vector startPos = hardware.drivetrain.robotPos;
        if (goldPos == 1)
            goToIntake(fieldMap.get(minerals[0]), pow);
        else if (goldPos == 2)
            goToIntake(fieldMap.get(minerals[1]), pow);
        else if (goldPos == 3) {
            goToIntake(fieldMap.get(minerals[2]), pow);
        }

        // Back up if necessary
        if (backup) {
            if (!reverse)
                hardware.drivetrain.driveDistance(-1, hardware.drivetrain.robotPos.distanceFrom(startPos) * 3/4, 0.6);
            else
                hardware.drivetrain.driveDistance(-1, hardware.drivetrain.robotPos.distanceFrom(startPos), 0.6);
            hardware.drivetrain.updatePosFromEncoders();
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

    public void releaseMarker(int alliance) throws InterruptedException {
        //hardware.intake.releaseMinerals(0.3);
        if (alliance == AutonomousData.RED_ALLIANCE)
            hardware.drivetrain.faceAngle(90);
        else if (alliance == AutonomousData.BLUE_ALLIANCE)
            hardware.drivetrain.faceAngle(270);
        hardware.drivetrain.strafeForTime(-0.8, 1);

        hardware.markerArm.setPosition(hardware.MARKER_ARM_DOWN);
        Thread.sleep(500);
        hardware.markerArm.setPosition(hardware.MARKER_ARM_UP);
        //hardware.intake.flipIn();
    }

    public void driveToCrater(int alliance) throws InterruptedException {
        if (alliance == AutonomousData.RED_ALLIANCE) {
            hardware.drivetrain.faceAngle(180);
        } else if (alliance == AutonomousData.BLUE_ALLIANCE) {
            hardware.drivetrain.faceAngle(0);
        }

        hardware.drivetrain.strafeForTime(-1, 1.5);
        hardware.drivetrain.driveDistance(1, fieldMap.SQUARE_LENGTH * 4, 1);
        hardware.intake.flipOut();
    }

    public void backupToLander(int distance) throws InterruptedException {
        hardware.drivetrain.encoderSetup();

        double startDistance = hardware.rangeSensor.getDistance(DistanceUnit.INCH);
        double currDistance = hardware.rangeSensor.getDistance(DistanceUnit.INCH);
        double startPow = 1.0; // starting power was 1.0
        double pow; // power applied to motors
        double prop; // proportion of angle completed

        while (currDistance > distance && autoRunning()) {
            prop = (currDistance - distance) / (startDistance - distance);
            pow = -startPow * Math.pow((prop - 1), 3);

            // Apply power to motors and update currDistance
            hardware.drivetrain.setPowers(-pow, -pow, 0);
            currDistance = hardware.rangeSensor.getDistance(DistanceUnit.INCH);
        }
        hardware.drivetrain.setPowers(0, 0, 0);

        // Updates the robot angle based on turn
        hardware.drivetrain.updateAngleFromIMU();
    }

    // Used to break all while loops when an opmode stops
    private boolean autoRunning() {
        return System.currentTimeMillis() - startTime <= AutonomousData.TIME_LIMIT && !autonomous.isStopRequested();
    }
}
