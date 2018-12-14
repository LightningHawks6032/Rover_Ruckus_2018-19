/**
 * This tests mineral sampling in the fourth quadrant (red depot side). For TESTING purposes ONLY.
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

// @Autonomous(name="Mineral Sampling", group="Linear Opmode")
public class Robot1_MineralSampling extends LinearOpMode {
    // Declare hardware
    private Robot1_Hardware hardware;
    private GoldAlignDetector detector;
    private FieldMap fieldMap = new FieldMap();


    public void runOpMode() throws InterruptedException {
        hardware = new Robot1_Hardware(hardwareMap, gamepad1, true);
        hardware.initHardware();

        detector = hardware.mineralDetector;
        detector.setupDetector(hardwareMap, 1); // Camera Index: 0 for back camera, 1 for front camera
        hardware.drivetrain.setRobotPos(fieldMap.SQUARE_LENGTH, -fieldMap.SQUARE_LENGTH); // We aren't testing nav target detection here, so assume this position
        hardware.drivetrain.setRobotAngle(315);

        waitForStart();

        telemetry.addLine("Mineral Sampling");
        telemetry.update();
        //turnToGold();

        positionalMineralSampling();
        goTo(fieldMap.get(FieldElement.RED_DEPOT), 0.8);

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
            //telemetry.addLine("Aligned with middle mineral");
            //telemetry.addData("ra", hardware.drivetrain.robotAngle);
            //telemetry.update();
            return 2;
        }

        hardware.drivetrain.turn(30, false);
        sleep(500);
        if (detector.getAligned()) {
            //telemetry.addLine("Aligned with left mineral");
            //telemetry.addData("ra", hardware.drivetrain.robotAngle);
            //telemetry.update();
            hardware.drivetrain.turn(30, true);
            return 1;
        }

        hardware.drivetrain.turn(60, true);
        sleep(500);
        if (detector.getAligned()) {
            //telemetry.addLine("Aligned with right mineral");
            //telemetry.addData("ra", hardware.drivetrain.robotAngle);
            //telemetry.update();
            hardware.drivetrain.turn(30, false);
            return 3;
        }

        hardware.drivetrain.turn(30, false);
        //telemetry.addData("ra", hardware.drivetrain.robotAngle);


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


        // Determine what angle to turn
        int tempRobotAngle = hardware.drivetrain.robotAngle > 180 ? -(360 - hardware.drivetrain.robotAngle) : hardware.drivetrain.robotAngle;
        telemetry.addData("temp angle", tempRobotAngle);
        telemetry.update();
        if (tempRobotAngle * theta < 0) {
            if (Math.abs(tempRobotAngle) + Math.abs(theta) < 180) {
                if (tempRobotAngle > theta)
                    turn(Math.abs(tempRobotAngle) + Math.abs(theta), true);
                else
                    turn(Math.abs(tempRobotAngle) + Math.abs(theta), false);
            }
            else {
                if (tempRobotAngle > theta)
                    turn(360 - (Math.abs(theta) + Math.abs(tempRobotAngle)), false);
                else
                    turn(360 - (Math.abs(theta) + Math.abs(tempRobotAngle)), true);
            }
        }
        else if (tempRobotAngle != theta) {
            if (tempRobotAngle < theta)
                turn(theta - tempRobotAngle, false);
            else
                turn(tempRobotAngle - theta, true);
        }

        hardware.drivetrain.driveDistance(1, location.distanceFrom(hardware.drivetrain.robotPos), pow);
        hardware.drivetrain.robotPos = location;
    }

    public void turn(int degrees, boolean right) {
        hardware.drivetrain.gyroSensor.zero();
        hardware.drivetrain.encoderSetup();

        int currAngle = Math.abs(hardware.drivetrain.gyroSensor.getAngle()); // Use getAngle() because it returns angle robot has turned from origin
        double pow = 1; // power applied to motors

        while (currAngle < degrees) {
            pow = (double) (degrees - currAngle) / degrees * 0.6 + 0.1;

            // Apply power to motors and update currAngle
            if (right) {
                hardware.drivetrain.setPowers(pow, -pow, 0);
            } else {
                hardware.drivetrain.setPowers(-pow, pow, 0);
            }
            currAngle = Math.abs(hardware.drivetrain.gyroSensor.getAngle());

            telemetry.addData("curr angle", currAngle);
            telemetry.addData("degree", degrees);
            telemetry.addData("power left", hardware.drivetrain.leftMotor.getPower());
            telemetry.addData("power right", hardware.drivetrain.rightMotor.getPower());
            telemetry.update();
        }
        hardware.drivetrain.setPowers(0, 0, 0);

        // Updates the robot angle based on turn
        hardware.drivetrain.setRobotAngle((360 + hardware.drivetrain.robotAngle - hardware.drivetrain.gyroSensor.getAngle()) % 360);

        /*if (right)
            setRobotAngle((360 + robotAngle - Math.abs(degrees)) % 360);
        else
            setRobotAngle((360 + robotAngle + Math.abs(degrees)) % 360);*/
    }

}
