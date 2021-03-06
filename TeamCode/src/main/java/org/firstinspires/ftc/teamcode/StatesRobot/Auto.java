package org.firstinspires.ftc.teamcode.StatesRobot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.AutonomousData;
import org.firstinspires.ftc.teamcode.FieldMapping.FieldElement;
import org.firstinspires.ftc.teamcode.FieldMapping.FieldMap;
import org.firstinspires.ftc.teamcode.FieldMapping.Vector;
import org.firstinspires.ftc.teamcode.Hardware.QualBot_Hardware;
import org.firstinspires.ftc.teamcode.Hardware.StatesBot_Hardware;
import org.firstinspires.ftc.teamcode.Hardware.StatesBot_Intake;
import org.firstinspires.ftc.teamcode.Hardware.StatesBot_Outtake;
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
        double defaultOffset = 17; // originally 15
        if ((int) hardware.rangeSensor.getDistance(DistanceUnit.INCH) == 0) { // if range sensor is not working
            hardware.drivetrain.setRobotPos(new Vector(quadrant == 1 || quadrant == 4 ? defaultOffset : -defaultOffset, quadrant < 3 ? defaultOffset : -defaultOffset));
            hardware.drivetrain.updateAngleFromIMU();
            return;
        }

        hardware.drivetrain.faceAngle(startTheta(quadrant)); // used to be commented out
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

    // Returns the starting temporary angle (theta) of the robot dependent on its starting quadrant
    public int startTheta(int quadrant) {
        if (quadrant <= 2)
            return startAngle(quadrant);
        return -(45 + 90 * (4 - quadrant));
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

    /**
     * Create an array of the minerals, represented as FieldElements, in a particular sampling field.
     * @param quadrant : the quadrant where the sampling field is happens (1-4)
     * @return the array of minerals, from left to right from the POV of the robot.
     */
    private FieldElement[] samplingField(int quadrant) {
        FieldElement[] minerals = new FieldElement[3];
        switch (quadrant) {
            case 1:
                minerals[0] = FieldElement.BLUE_CRATER_LEFT_MINERAL;
                minerals[1] = FieldElement.BLUE_CRATER_MIDDLE_MINERAL;
                minerals[2] = FieldElement.BLUE_CRATER_RIGHT_MINERAL;
                break;
            case 2:
                minerals[0] = FieldElement.BLUE_DEPOT_LEFT_MINERAL;
                minerals[1] = FieldElement.BLUE_DEPOT_MIDDLE_MINERAL;
                minerals[2] = FieldElement.BLUE_DEPOT_RIGHT_MINERAL;
                break;
            case 3:
                minerals[0] = FieldElement.RED_CRATER_LEFT_MINERAL;
                minerals[1] = FieldElement.RED_CRATER_MIDDLE_MINERAL;
                minerals[2] = FieldElement.RED_CRATER_RIGHT_MINERAL;
                break;
            case 4:
                minerals[0] = FieldElement.RED_DEPOT_LEFT_MINERAL;
                minerals[1] = FieldElement.RED_DEPOT_MIDDLE_MINERAL;
                minerals[2] = FieldElement.RED_DEPOT_RIGHT_MINERAL;
                break;
        }

        return minerals;
    }

    public void landOnField(int quadrant) throws InterruptedException {
        // Land on field
        hardware.outtake.verticalSlideUp();

        // Set up IMU readings and robot angle
        autonomous.sleep(500);
        hardware.drivetrain.setInitialIMUHeading();
        hardware.drivetrain.setInitialRobotAngle(startAngle(quadrant));

        // Move away from lander
        hardware.drivetrain.driveDistance(1, 8, 0.8);
        hardware.drivetrain.updateAngleFromIMU(); // added
        //hardware.outtake.verticalSlideDown();
    }

    /**
     * Moves vertical slide while moving the horizontal slide. If vertical slide is not fully extended/retracted by end, moves the rest of the slide.
     * @param vertUp : true if the vertical slide should go up to its max, false if it should go down to its min
     * @param hsTarget : the encoder or linear distance target for the horizontal slide
     * @param linDistance : true if the hsTarget represents a linear distance, false if it represents encoder ticks
     */
    private void vertAndExtend(boolean vertUp, double hsTarget, boolean linDistance) {
        StatesBot_Intake i = hardware.intake;
        StatesBot_Outtake o = hardware.outtake;
        i.slideEncoder.runToPosition();
        if (linDistance) {
            i.slideEncoder.setTarget(hsTarget);
            i.horizontalSlide.setPower(i.slideEncoder.linDistance() > hsTarget ? -1 : 1);
        }
        else {
            i.slideEncoder.setEncoderTarget((int) hsTarget);
            i.horizontalSlide.setPower(i.slideEncoder.getEncoderCount() > hsTarget ? -1 : 1);
        }
        while (i.horizontalSlide.isBusy() && autoRunning()) {
            // WAIT - Motor is busy
            int vertEncoder = (o.leftVertEncoder.getEncoderCount() + o.rightVertEncoder.getEncoderCount()) / 2;
            if (!vertUp && vertEncoder > o.VERTICAL_SLIDE_MIN) {
                o.leftVertical.setPower(-1);
                o.rightVertical.setPower(-1);
            } else if (vertUp && vertEncoder < o.VERTICAL_SLIDE_MAX) {
                o.leftVertical.setPower(1);
                o.rightVertical.setPower(1);
            } else {
                o.leftVertical.setPower(0);
                o.rightVertical.setPower(0);
            }
        }
        i.horizontalSlide.setPower(0);
        i.slideEncoder.runWithout();
        if (vertUp) o.verticalSlideUp(); // if the vertical slide still has not finished extending
        else o.verticalSlideDown(); // if the vertical slide still has not finished retracting
    }

    /**
     * Robot performs mineral sampling using the horizontal slide (rather than driving to the gold mineral)
     * @param goldPos : the position of the gold mineral determined while the robot is hanging at beginning of autonomous
     * @param quadrant : the quadrant where the mineral sampling happens (1-4)
     * @param intake : true if the robot should intake the gold mineral (depot side), false otherwise (crater side)
     * @throws InterruptedException
     */
    public void sampleWithSlide(int goldPos, int quadrant, boolean intake) throws InterruptedException {
        // Generates minerals to choose from
        FieldElement[] minerals = samplingField(quadrant);

        // Find chosen mineral
        FieldElement chosenMineral = minerals[1]; // default
        if (goldPos == 1) {
            chosenMineral = minerals[0];
        } else if (goldPos == 2) {
            chosenMineral = minerals[1];
        } else if (goldPos == 3) {
            chosenMineral = minerals[2];
        }
        double distFromMineral = fieldMap.get(chosenMineral).distanceFrom(hardware.drivetrain.robotPos) - 10; // -10 accounts for distance of front of horizontal slide from robot center

        // If we're on depot side, we run our slide to a fraction of the distance for time saving (because this would be after marker scoring)
        if (intake && quadrant % 2 == 0) {
            hardware.intake.runSlideTo(0.3 * distFromMineral);
        }

        hardware.drivetrain.face(fieldMap.get(chosenMineral));
        hardware.intake.flipOut(true);

        // If we're intaking, run harvester and run  horizontal slide to the mineral; otherwise, run  horizontal slide less while lowering vertical slide
        if (intake) {
            hardware.intake.harvest();
            hardware.intake.runSlideTo(quadrant % 2 == 1 ? 0.6 * distFromMineral : distFromMineral);
        } else {
            vertAndExtend(false, 0.7 * distFromMineral, true);
        }
        hardware.intake.stopHarvester();
        hardware.intake.flipIn(false);
    }

    public void doubleSampleWithSlide(int goldPos, int alliance, boolean intake) throws InterruptedException {
        /*FieldElement[] minerals = samplingField(quadrant);

        // Find chosen mineral
        FieldElement chosenMineral = minerals[1]; // default
        if (goldPos == 1) {
            chosenMineral = minerals[0];
        } else if (goldPos == 2) {
            chosenMineral = minerals[1];
        } else if (goldPos == 3) {
            chosenMineral = minerals[2];
        }*/

        hardware.drivetrain.driveDistance(1, 8, 1.0); // was 8 inch
        hardware.drivetrain.updatePosAfterDrive(1);
        int thetaFace = alliance == AutonomousData.BLUE_ALLIANCE ? -135 : 45;
        hardware.drivetrain.faceAngle(thetaFace);
        hardware.drivetrain.faceAngle(thetaFace);

        if (goldPos == 3) {
            hardware.intake.flipOut(true);
            hardware.intake.extendHorizontalSlide(0.3);
            hardware.intake.flipIn(true);
        } else if (goldPos == 2) {
            hardware.intake.extendHorizontalSlide(0.4);
            hardware.intake.flipOut(true);
            hardware.intake.extendHorizontalSlide(0.7);
            hardware.intake.flipIn(true);
        } else if (goldPos == 1) {
            hardware.drivetrain.driveDistance(1, 4, 0.8);
            hardware.intake.extendHorizontalSlide(0.7);
            hardware.intake.flipOut(true);
            hardware.intake.extendHorizontalSlide(1);
            hardware.intake.flipIn(true);
            hardware.drivetrain.driveDistance(-1, 4, 0.8);
        }

        hardware.intake.retractHorizontalSlide();
    }

    public void releaseMarkerWithSlide(int quadrant) throws InterruptedException {
        // If starting on crater side, we have to drive to get closer to depot; otherwise, simply extend while lowering vertical slide
        if (quadrant % 2 == 1) {
            //hardware.outtake.verticalSlideDown();
            hardware.drivetrain.goToLerp(AutonomousData.FIELD_MAP.get(quadrant == 1 ? FieldElement.FRONT_OF_BLUE_ROVER : FieldElement.FRONT_OF_RED_FOOTPRINT), 1.2); // 0.6, 0.8 worked
            hardware.drivetrain.faceAngle(quadrant == 1 ? 170 : -10);
            hardware.drivetrain.driveDistance(1, 6, 1.0); // was 5 inch
            hardware.drivetrain.updatePosAfterDrive(1);
        }
        vertAndExtend(false, hardware.intake.HORIZONTAL_SLIDE_MAX, false);
        hardware.intake.flipOut(true);
        hardware.intake.releaseForTime(false, 1.0); // could be minimized?
        hardware.intake.flipIn(false);
    }

    public void scoreInLander(int quadrant) throws InterruptedException {
        hardware.intake.release();
        if (quadrant % 2 != 1) hardware.drivetrain.faceAngle(startTheta(quadrant));
        hardware.drivetrain.driveDistance(-1, 5, 0.7);
        hardware.intake.stopHarvester();

        if (quadrant % 2 == 1) {
            //hardware.drivetrain.turn(30, true); // if on crater side, we have to turn to get mineral in gold cargo hold
            //hardware.drivetrain.driveDistance(-1, 3, 1.0);
            vertAndExtend(true, hardware.intake.HORIZONTAL_SLIDE_MAX, false); // extend horizontal slide to get parking points
        } else {
            hardware.outtake.verticalSlideUp();
        }

        hardware.outtake.dump();
        hardware.outtake.verticalSlideDown();
    }

    public void parkInCraterFromLander(int alliance, boolean ourCrater) throws InterruptedException {
        hardware.drivetrain.driveDistance(1, 6, 0.6);
        hardware.drivetrain.updatePosAfterDrive(1);

        FieldElement navTarget; // Nav Target robot should drive towards
        int thetaFace; // Angle the robot should face after reaching crater to extend horizontal slide
        if (alliance == AutonomousData.RED_ALLIANCE) {
            navTarget = ourCrater ? FieldElement.FRONT_OF_RED_FOOTPRINT : FieldElement.FRONT_OF_FRONT_CRATERS;
            thetaFace = ourCrater ? 180 : 90;
        }
        else {
            navTarget = ourCrater ? FieldElement.FRONT_OF_BLUE_ROVER : FieldElement.FRONT_OF_BACK_SPACE;
            thetaFace = ourCrater ? 0 : -90;
        }

        hardware.drivetrain.goTo(fieldMap.get(navTarget), 0.6);
        hardware.drivetrain.faceAngle(thetaFace);
        hardware.drivetrain.faceAngle(thetaFace);
        hardware.intake.extendHorizontalSlide(0.7);
    }

    public void parkInCraterFromDoubleSample(int alliance) throws InterruptedException {
        int thetaFace = alliance == AutonomousData.BLUE_ALLIANCE ? 10 : -170;
        hardware.drivetrain.faceAngle(thetaFace);
        hardware.drivetrain.faceAngle(thetaFace);
        //hardware.drivetrain.driveDistance(1, AutonomousData.FIELD_MAP.HALF_SQUARE_LENGTH, 1);
        hardware.intake.extendHorizontalSlide(1);
    }

    public void backToStartingPosition(int alliance) throws InterruptedException {
        hardware.drivetrain.driveDistance(-1, 6, 1.0); // was 5 inch
        hardware.drivetrain.updatePosAfterDrive(-1);
        double coordinate = 0.8*AutonomousData.FIELD_MAP.SQUARE_LENGTH;
        hardware.drivetrain.goToBackwardsLerp(new Vector(alliance == AutonomousData.RED_ALLIANCE ? -coordinate : coordinate, alliance == AutonomousData.RED_ALLIANCE ? -coordinate : coordinate), 1.2);
        hardware.drivetrain.faceAngle(startTheta(alliance == AutonomousData.BLUE_ALLIANCE ? 1 : 3));
    }

    /**
     * Backs up to the lander using the range sensor
     * @param distance : distance, in inches, the back of the robot should be from the lander wall
     * @throws InterruptedException
     */
    public void backupToLander(int distance) throws InterruptedException {
        hardware.drivetrain.encoderSetup();

        Thread.sleep(300);
        double startDistance = hardware.rangeSensor.getDistance(DistanceUnit.INCH);
        double currDistance = hardware.rangeSensor.getDistance(DistanceUnit.INCH);
        double startPow = 0.6; // starting power was 1.0
        double pow; // power applied to motors
        double prop; // proportion of angle completed

        while (currDistance > distance && autoRunning()) {
            prop = (currDistance - distance) / (startDistance - distance);
            pow = -startPow * Math.pow((prop - 1), 1);

            // Apply power to motors and update currDistance
            hardware.drivetrain.setPowers(-pow, -pow, -pow, -pow);
            currDistance = hardware.rangeSensor.getDistance(DistanceUnit.INCH);
        }
        hardware.drivetrain.setPowers(0, 0, 0,0);

        autonomous.telemetry.addData("Range Sensor Dist", hardware.rangeSensor.getDistance(DistanceUnit.INCH));
        autonomous.telemetry.update();

        // Updates the robot angle based on turn
        hardware.drivetrain.updateAngleFromIMU();
    }

    // Used to break all while loops when an opmode stops
    public boolean autoRunning() {
        return System.currentTimeMillis() - startTime <= AutonomousData.TIME_LIMIT && !autonomous.isStopRequested();
    }
}
