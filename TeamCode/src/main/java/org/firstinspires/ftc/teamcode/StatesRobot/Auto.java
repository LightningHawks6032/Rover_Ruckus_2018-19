package org.firstinspires.ftc.teamcode.StatesRobot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.AutonomousData;
import org.firstinspires.ftc.teamcode.FieldMapping.FieldElement;
import org.firstinspires.ftc.teamcode.FieldMapping.FieldMap;
import org.firstinspires.ftc.teamcode.FieldMapping.Vector;
import org.firstinspires.ftc.teamcode.Hardware.QualBot_Hardware;
import org.firstinspires.ftc.teamcode.Hardware.StatesBot_Hardware;
import org.firstinspires.ftc.teamcode.Vision.Detectors.GoldAlignDetector;
import org.firstinspires.ftc.teamcode.Vision.Detectors.NavTargetDetector;

public class Auto {
    private LinearOpMode autonomous;
    private StatesBot_Hardware hardware;
    private FieldMap fieldMap = new FieldMap();
    private GoldAlignDetector mineralDetector;
    private NavTargetDetector navTargetDetector;
    private long startTime;

    // Constructor instantiates hardware and setups mineral detector
    public Auto(LinearOpMode auto, StatesBot_Hardware hardware) {
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
                hardware.drivetrain.setRobotPos(landerPos.sum(new Vector(coordinateOffset+2, coordinateOffset+2)));
                break;
            case 2:
                landerPos = fieldMap.get(FieldElement.QUAD_2_LANDER_WALL);
                hardware.drivetrain.setRobotPos(landerPos.sum(new Vector(-coordinateOffset, coordinateOffset)));
                break;
            case 3:
                landerPos = fieldMap.get(FieldElement.QUAD_3_LANDER_WALL);
                hardware.drivetrain.setRobotPos(landerPos.sum(new Vector(-coordinateOffset-2, -coordinateOffset-2)));
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
        autonomous.sleep(1000);
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
            hardware.drivetrain.gyro.zero();

            // Debug
            //autonomous.telemetry.addData("Robot Pos", hardware.drivetrain.robotPos.toString());
            autonomous.telemetry.addData("Robot Angle From Nav", hardware.drivetrain.robotAngle);
            autonomous.telemetry.update();
        }
    }

    public void landOnField(int quadrant) {
        // Land on field
        hardware.outtake.verticalSlideUp();

        // Set up IMU readings and robot angle
        autonomous.sleep(500);
        hardware.drivetrain.setInitialIMUHeading();
        hardware.drivetrain.setInitialRobotAngle(startAngle(quadrant));

        // Move away from lander
        hardware.drivetrain.driveDistance(1, 8, 0.6);
        hardware.outtake.verticalSlideDown();
    }

    public void driveAndLowerSlide(int direction, double distance, double pow) throws InterruptedException {
        hardware.drivetrain.leftFrontEncoder.reset();
        hardware.drivetrain.leftBackEncoder.reset();
        hardware.drivetrain.rightFrontEncoder.reset();
        hardware.drivetrain.rightBackEncoder.reset();

        hardware.drivetrain.leftBackEncoder.runToPosition();
        hardware.drivetrain.rightBackEncoder.runToPosition();
        hardware.drivetrain.leftFrontEncoder.runToPosition();
        hardware.drivetrain.rightFrontEncoder.runToPosition();

        hardware.drivetrain.leftBackEncoder.setTarget(direction * distance);
        hardware.drivetrain.rightBackEncoder.setTarget(direction * distance);
        hardware.drivetrain.leftFrontEncoder.setTarget(direction * distance);
        hardware.drivetrain.rightFrontEncoder.setTarget(direction * distance);

        hardware.drivetrain.setPowers(direction * pow, direction * pow,direction * pow,direction * pow);

        while (hardware.drivetrain.leftFront.isBusy() && hardware.drivetrain.rightFront.isBusy() && hardware.drivetrain.leftBack.isBusy() && hardware.drivetrain.rightBack.isBusy() && autoRunning()) {
            double encoderDistance = (hardware.drivetrain.leftBackEncoder.linDistance() + hardware.drivetrain.rightBackEncoder.linDistance() + hardware.drivetrain.leftFrontEncoder.linDistance() + hardware.drivetrain.rightFrontEncoder.linDistance()) / 4;
            double prop = encoderDistance / distance;
            double newPow = -(pow - 0.1) * Math.pow(prop - 1, 3) + 0.1;
            hardware.drivetrain.setPowers(direction * newPow, direction * newPow,direction * newPow, direction * newPow );

            int slideEncoderVal = (hardware.outtake.leftVertEncoder.getEncoderCount() + hardware.outtake.rightVertEncoder.getEncoderCount()) / 2; // average
            if (slideEncoderVal > hardware.outtake.VERTICAL_SLIDE_MIN) {
                hardware.outtake.leftVertical.setPower(-1);
                hardware.outtake.rightVertical.setPower(-1);
            }
        }

        hardware.drivetrain.setPowers(0, 0, 0,0);
        hardware.drivetrain.leftFrontEncoder.runWithout();
        hardware.drivetrain.leftBackEncoder.runWithout();
        hardware.drivetrain.rightFrontEncoder.runWithout();
        hardware.drivetrain.rightBackEncoder.runWithout();
    }

    private void goToIntake(Vector location, double pow) throws InterruptedException {
        hardware.drivetrain.face(location); // Turn to face location
        hardware.intake.flipOut();
        hardware.intake.harvest();
        hardware.drivetrain.driveDistance(1, location.distanceFrom(hardware.drivetrain.robotPos), pow); // Drive to location
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
        autonomous.sleep(500);
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
            hardware.drivetrain.goTo(fieldMap.get(minerals[0]), 1);
        else if (goldPos == 2)
            hardware.drivetrain.goTo(fieldMap.get(minerals[1]), 1);
        else if (goldPos == 3) {
            hardware.drivetrain.goTo(fieldMap.get(minerals[2]), 1);
        }

        // Back up if necessary
        if (backup) {
            if (!reverse)
                hardware.drivetrain.driveDistance(-1, hardware.drivetrain.robotPos.distanceFrom(startPos) * 3/4, 1);
            else
                hardware.drivetrain.driveDistance(-1, hardware.drivetrain.robotPos.distanceFrom(startPos), 1);
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
        double pow = backup ? 0.5 : 1;

        // Go to Gold
        Vector startPos = hardware.drivetrain.robotPos;
        if (goldPos == 1) {
            //hardware.drivetrain.goTo(fieldMap.get(minerals[0]), pow);
            goToIntake(fieldMap.get(minerals[0]), pow);
        } else if (goldPos == 2) {
            //hardware.drivetrain.goTo(fieldMap.get(minerals[1]), pow);
            goToIntake(fieldMap.get(minerals[1]), pow);
        } else if (goldPos == 3) {
            //hardware.drivetrain.goTo(fieldMap.get(minerals[2]), pow);
            goToIntake(fieldMap.get(minerals[2]), pow);
        }

        Thread.sleep(500);

        // Back up if necessary
        if (backup) {
            if (!reverse)
                hardware.drivetrain.driveDistance(-1, hardware.drivetrain.robotPos.distanceFrom(startPos) * 3/4, 1);
            else
                hardware.drivetrain.driveDistance(-1, hardware.drivetrain.robotPos.distanceFrom(startPos), 1);
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

    // Turns towards depot, sends horizontal slide forward, de-harvests
    public void releaseMarker(int alliance) throws InterruptedException {
        //hardware.intake.releaseMinerals(0.3);
        if (alliance == AutonomousData.RED_ALLIANCE)
            hardware.drivetrain.faceAngle(45);
        else if (alliance == AutonomousData.BLUE_ALLIANCE)
            hardware.drivetrain.faceAngle(225);
        hardware.drivetrain.strafeForTime(-1, 0.8);


        //hardware.intake.flipIn();
    }

    public void driveToCrater(int alliance) throws InterruptedException {
        if (alliance == AutonomousData.RED_ALLIANCE) {
            hardware.drivetrain.faceAngle(190);
        } else if (alliance == AutonomousData.BLUE_ALLIANCE) {
            hardware.drivetrain.faceAngle(10);
        }

        hardware.drivetrain.driveDistance(1, fieldMap.SQUARE_LENGTH * 3.2, 3);
        hardware.intake.horizontalSlide.setPower(1);
        Thread.sleep(800);
        hardware.intake.horizontalSlide.setPower(0);
    }

    public void driveToOtherCrater(int alliance) throws InterruptedException {
        if (alliance == AutonomousData.RED_ALLIANCE) {
            hardware.drivetrain.faceAngle(60);
        } else if (alliance == AutonomousData.BLUE_ALLIANCE) {
            hardware.drivetrain.faceAngle(240);
        }

        hardware.drivetrain.driveDistance(1, fieldMap.SQUARE_LENGTH * 3.2, 3);
        hardware.intake.horizontalSlide.setPower(1);
        Thread.sleep(800);
        hardware.intake.horizontalSlide.setPower(0);
    }

    public void backupToLander(int distance) throws InterruptedException {
        //hardware.drivetrain.encoderSetup();

        double startDistance = hardware.rangeSensor.getDistance(DistanceUnit.INCH);
        double currDistance = hardware.rangeSensor.getDistance(DistanceUnit.INCH);
        double startPow = 1.0; // starting power was 1.0
        double pow; // power applied to motors
        double prop; // proportion of angle completed

        while (currDistance > distance && autoRunning()) {
            prop = (currDistance - distance) / (startDistance - distance);
            pow = -startPow * Math.pow((prop - 1), 1);

            // Apply power to motors and update currDistance
            hardware.drivetrain.setPowers(-pow, -pow,-pow, -pow);
            currDistance = hardware.rangeSensor.getDistance(DistanceUnit.INCH);
        }
        hardware.drivetrain.setPowers(0, 0, 0,0);

        // Updates the robot angle based on turn
        hardware.drivetrain.updateAngleFromIMU();
    }

    // Used to break all while loops when an opmode stops
    private boolean autoRunning() {
        return System.currentTimeMillis() - startTime <= AutonomousData.TIME_LIMIT && !autonomous.isStopRequested();
    }
}
