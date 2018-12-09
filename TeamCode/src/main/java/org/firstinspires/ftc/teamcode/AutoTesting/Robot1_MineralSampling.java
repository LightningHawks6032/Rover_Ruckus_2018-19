/**
 * This tests mineral sampling in the fourth quadrant (red depot side)
 */

package org.firstinspires.ftc.teamcode.AutoTesting;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FieldMapping.FieldElement;
import org.firstinspires.ftc.teamcode.FieldMapping.FieldMap;
import org.firstinspires.ftc.teamcode.FieldMapping.Vector;
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

        detector = new GoldAlignDetector(hardware.ROBOT_CENTER_X, 250, true); // Create detector
        detector.setupDetector(hardwareMap, 1); // Camera Index: 0 for back camera, 1 for front camera
        hardware.drivetrain.setRobotPos(fieldMap.HALF_SQUARE_LENGTH, -fieldMap.HALF_SQUARE_LENGTH); // We aren't testing nav target detection here, so assume this position
        hardware.drivetrain.setRobotAngle(315);

        waitForStart();

        telemetry.addLine("Mineral Sampling");
        telemetry.update();
        //turnToGold();

        positionalMineralSampling();

        detector.disable();
    }

    private void nonPositionalMineralSampling() {
        int goldPos = findGold();

        telemetry.addLine("Moving to hit mineral");
        telemetry.update();
        if (goldPos == 1) {
            telemetry.addLine("Driving to left mineral");
            telemetry.update();
            hardware.drivetrain.driveDistance(1, 30, 0.6);
            //hardware.drivetrain.goTo(fieldMap.get(FieldElement.RED_DEPOT_LEFT_MINERAL), 0.6);
        } else if (goldPos == 2) {
            telemetry.addLine("Driving to middle mineral");
            telemetry.update();
            hardware.drivetrain.driveDistance(1, 30, 0.6);
        } else if (goldPos == 3) {
            telemetry.addLine("Driving to right mineral");
            telemetry.update();
            hardware.drivetrain.driveDistance(1, 30, 0.6);
        }
    }

    private void positionalMineralSampling() {
        int goldPos = findGoldForPosition();

        telemetry.addData("gyro", hardware.drivetrain.getGyro().getAngle());
        telemetry.update();

        //telemetry.addLine("Moving to hit mineral");
        //telemetry.update();
        if (goldPos == 1) {
            //telemetry.addLine("Driving to left mineral");
            //telemetry.update();
            goTo(fieldMap.get(FieldElement.RED_DEPOT_LEFT_MINERAL), 0.6);
        } else if (goldPos == 2) {
            //telemetry.addLine("Driving to middle mineral");
            //telemetry.update();
            goTo(fieldMap.get(FieldElement.RED_DEPOT_MIDDLE_MINERAL), 0.6);
        } else if (goldPos == 3) {
            //telemetry.addLine("Driving to right mineral");
            //telemetry.update();
            goTo(fieldMap.get(FieldElement.RED_DEPOT_RIGHT_MINERAL), 0.6);
        }
    }

    private int findGold() {
        sleep(1000);
        if (detector.getAligned()) {
            telemetry.addLine("Aligned with middle mineral");
            telemetry.update();
            return 2;
        }

        hardware.drivetrain.turn(30, false);
        sleep(500);
        if (detector.getAligned()) {
            telemetry.addLine("Aligned with left mineral");
            telemetry.update();
            return 1;
        }

        hardware.drivetrain.turn(60, true);
        sleep(500);
        if (detector.getAligned()) {
            telemetry.addLine("Aligned with right mineral");
            telemetry.update();
            return 3;
        }

        hardware.drivetrain.turn(30, false);

        return 2;
    }

    private int findGoldForPosition() {
        sleep(1000);
        if (detector.getAligned()) {
            telemetry.addLine("Aligned with middle mineral");
            telemetry.update();
            return 2;
        }

        hardware.drivetrain.turn(30, false);
        sleep(200);
        if (detector.getAligned()) {
            telemetry.addLine("Aligned with left mineral");
            telemetry.update();
            hardware.drivetrain.turn(30, true);
            return 1;
        }

        hardware.drivetrain.turn(60, true);
        sleep(200);
        if (detector.getAligned()) {
            telemetry.addLine("Aligned with right mineral");
            telemetry.update();
            hardware.drivetrain.turn(30, false);
            return 3;
        }

        hardware.drivetrain.turn(30, false);

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


    private void goTo(Vector location, double pow) {
        double radiansToTurn = Math.atan2(location.getY() - hardware.drivetrain.robotPos.getY(), location.getX() - hardware.drivetrain.robotPos.getX());
        int theta = hardware.drivetrain.getGyro().convertToDegrees(radiansToTurn);
        telemetry.addData("theta", theta);
        telemetry.update();


        // Determine what angle to turn
        int tempRobotAngle = -45;
        if (tempRobotAngle * theta < 0) {
            if (Math.abs(tempRobotAngle) + Math.abs(theta) < 180) {
                if (tempRobotAngle > theta)
                    hardware.drivetrain.turn(Math.abs(tempRobotAngle) + Math.abs(theta), true);
                else
                    hardware.drivetrain.turn(Math.abs(tempRobotAngle) + Math.abs(theta), false);
            }
            else {
                if (tempRobotAngle > theta)
                    hardware.drivetrain.turn(360 - (Math.abs(theta) + Math.abs(tempRobotAngle)), false);
                else
                    hardware.drivetrain.turn(360 - (Math.abs(theta) + Math.abs(tempRobotAngle)), true);
            }
        }
        else if (tempRobotAngle != theta) {
            if (tempRobotAngle < theta)
                hardware.drivetrain.turn(theta - tempRobotAngle, false);
            else
                hardware.drivetrain.turn(tempRobotAngle - theta, true);
        }

        hardware.drivetrain.driveDistance(1, location.distanceFrom(hardware.drivetrain.robotPos), pow);
        hardware.drivetrain.robotPos = location;
    }

}
