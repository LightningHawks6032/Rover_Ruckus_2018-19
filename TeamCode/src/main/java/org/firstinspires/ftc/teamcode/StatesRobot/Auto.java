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
        hardware.drivetrain.faceAngle(startTheta(quadrant));

        double distFromLander = hardware.rangeSensor.getDistance(DistanceUnit.INCH);
        double defaultOffset = 15;
        if ((int) distFromLander == 0) {
            hardware.drivetrain.setRobotPos(new Vector(quadrant == 1 || quadrant == 4 ? defaultOffset : -defaultOffset, quadrant < 3 ? defaultOffset : -defaultOffset));
            hardware.drivetrain.updateAngleFromIMU();
            return;
        }
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

    public void landOnField(int quadrant) throws InterruptedException {
        // Land on field
        hardware.outtake.verticalSlideUp();

        // Set up IMU readings and robot angle
        autonomous.sleep(500);
        hardware.drivetrain.setInitialIMUHeading();
        hardware.drivetrain.setInitialRobotAngle(startAngle(quadrant));

        // Move away from lander
        hardware.drivetrain.driveDistance(1, 8, 0.6);
        //hardware.outtake.verticalSlideDown();
    }

    public void lowerAndExtend() {
        StatesBot_Intake i = hardware.intake;
        StatesBot_Outtake o = hardware.outtake;
        i.slideEncoder.runToPosition();
        i.slideEncoder.setEncoderTarget(i.HORIZONTAL_SLIDE_MAX);
        i.horizontalSlide.setPower(1);
        while (i.horizontalSlide.isBusy() && autoRunning()) {
            // WAIT - Motor is busy
            int vertEncoder = (o.leftVertEncoder.getEncoderCount() + o.rightVertEncoder.getEncoderCount()) / 2;
            if (vertEncoder > o.VERTICAL_SLIDE_MIN) {
                o.leftVertical.setPower(-1);
                o.rightVertical.setPower(-1);
            } else {
                o.leftVertical.setPower(0);
                o.rightVertical.setPower(0);
            }
        }
        i.horizontalSlide.setPower(0);
        i.slideEncoder.runWithout();
        o.verticalSlideDown(); // if the vertical slide still has not finished retracting
    }

    /**
     * Robot performs mineral sampling using the horizontal slide (rather than driving to the gold mineral)
     * @param goldPos : the position of the gold mineral determined while the robot is hanging at beginning of autonomous
     * @param quadrant : the quadrant where the mineral sampling happens (1-4)
     * @param reverse : true if robot back is to lander, false if robot front is to lander
     * @throws InterruptedException
     */
    public void sampleWithSlide(int goldPos, int quadrant, boolean reverse) throws InterruptedException {
        // Generates minerals to choose from
        FieldElement[] minerals = samplingField(quadrant, reverse);

        // Go to Gold
        Vector startPos = hardware.drivetrain.robotPos;
        double distFromMineral;
        FieldElement chosenMineral = minerals[1]; // default
        if (goldPos == 1) {
            chosenMineral = minerals[0];
        } else if (goldPos == 2) {
            chosenMineral = minerals[1];
        } else if (goldPos == 3) {
            chosenMineral = minerals[2];
        }

        distFromMineral = fieldMap.get(chosenMineral).distanceFrom(startPos) - 10;
        hardware.intake.runSlideTo(0.3 * distFromMineral);
        hardware.drivetrain.face(fieldMap.get(chosenMineral));
        hardware.intake.flipOut(true);
        hardware.intake.harvest();
        hardware.intake.runSlideTo(1.1 * distFromMineral);
        hardware.intake.stopHarvester();
        hardware.intake.flipIn(false);
        hardware.intake.retractHorizontalSlide();
    }

    /**
     * Robot performs mineral sampling by driving toward gold mineral to displace it from starting location.
     * @param goldPos : the position of the gold mineral determined while the robot is hanging at beginning of autonomous
     * @param quadrant : the quadrant where the mineral sampling happens (1-4)
     * @param reverse : true if robot back is to lander, false if robot front is to lander
     * @param backup : true if we want robot to back up after knocking over gold mineral
     * @throws InterruptedException
     */
    public void sampleWithDrive(int goldPos, int quadrant, boolean reverse, boolean backup) throws InterruptedException {
        // Generates minerals to choose from
        FieldElement[] minerals = samplingField(quadrant, reverse);

        // Go to Gold
        Vector startPos = hardware.drivetrain.robotPos;
        if (goldPos == 1) {
            hardware.drivetrain.goTo(fieldMap.get(minerals[0]), 0.6);
        } else if (goldPos == 2) {
            hardware.drivetrain.goTo(fieldMap.get(minerals[1]), 0.6);
        } else if (goldPos == 3) {
            hardware.drivetrain.goTo(fieldMap.get(minerals[2]), 0.6);
        }

        Thread.sleep(500);

        // Back up if necessary
        if (backup) {
            if (!reverse)
                hardware.drivetrain.driveDistance(-1, hardware.drivetrain.robotPos.distanceFrom(startPos) * 3/4, 1);
            else
                hardware.drivetrain.driveDistance(-1, hardware.drivetrain.robotPos.distanceFrom(startPos), 1);
            hardware.drivetrain.updatePosAfterDrive(-1);
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

    public void releaseMarkerWithSlide() throws InterruptedException {
        lowerAndExtend();
        hardware.intake.flipOut(true);
        hardware.intake.releaseForTime(false, 1.0);
        hardware.intake.flipIn(false);
    }

    public void scoreInLander(int quadrant) throws InterruptedException {
        hardware.intake.release();
        hardware.drivetrain.faceAngle(startTheta(quadrant));
        //backupToLander(5);
        hardware.drivetrain.driveDistance(-1, 3, 0.6);
        hardware.intake.stopHarvester();
        hardware.outtake.verticalSlideUp();
        hardware.outtake.dump();
        hardware.outtake.verticalSlideDown();
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
    private boolean autoRunning() {
        return System.currentTimeMillis() - startTime <= AutonomousData.TIME_LIMIT && !autonomous.isStopRequested();
    }
}
