/**
 * This tests multiple different turn functions. Each turn algorithm models power with respect to proportion of turn completed or degrees left.
 */

package org.firstinspires.ftc.teamcode.TestOpmodes.AutoTesting;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutonomousData;
import org.firstinspires.ftc.teamcode.Hardware.OfficialBot_Hardware;
import org.firstinspires.ftc.teamcode.Hardware.OmniSlideDrive;

@Autonomous(name="Turning with Gyro (Algorithms Test)", group= AutonomousData.TEST_GROUP)
public class TurningWithGyro extends LinearOpMode {
    private OfficialBot_Hardware hardware;
    private OmniSlideDrive drivetrain;

    private final double START_POWER = 0.5;
    private int[] degreesToTest = {20, 45, 90, 135};

    private int sumOfLinearError = 0;
    private int sumOfQuadraticError = 0;
    private int sumOfCubicError = 0;
    private int sumOfLinearDegreesError = 0;

    public void runOpMode() {
        hardware = new OfficialBot_Hardware(hardwareMap, gamepad1, gamepad2, true);
        hardware.initHardware();
        drivetrain = hardware.drivetrain;
        waitForStart();

        for (int d : degreesToTest) {
            turnLinear(d, true);
            turnQuadratic(d, true);
            turnCubic(d, true);
            turnDegreesLeft(d, true);
        }

        telemetry.addData("Turn Linear Total Error", sumOfLinearError);
        telemetry.addData("Turn Quadratic Total Error", sumOfQuadraticError);
        telemetry.addData("Turn Cubic Total Error", sumOfCubicError);
        telemetry.addData("Turn Degrees Left Total Error", sumOfLinearDegreesError);
        telemetry.update();

        sleep(10000);

    }

    private void turnLinear(int degrees, boolean right) {
        drivetrain.gyroSensor.zero();
        drivetrain.encoderSetup();
        long startTime = System.currentTimeMillis();

        telemetry.addLine("Linear to Proportion Completed");
        telemetry.addData("Turn Degrees: ", degrees);
        telemetry.update();

        int currAngle = Math.abs(drivetrain.gyroSensor.getAngle()); // Use getAngle() because it returns angle robot has turned from origin
        double pow; // power applied to motors
        while (currAngle < degrees && !isStopRequested()) {
            pow = linearToProportion((double) currAngle / degrees);
            if (right)
                drivetrain.setPowers(pow, -pow, 0);
            else
                drivetrain.setPowers(-pow, pow, 0);
            currAngle = Math.abs(drivetrain.gyroSensor.getAngle());
        }
        drivetrain.setPowers(0, 0, 0);

        telemetry.addData("Gyro Sensor Reading", drivetrain.gyroSensor.getAngle());
        telemetry.addData("Error (Degrees)", drivetrain.gyroSensor.getAngle() - degrees);
        telemetry.addData("Turn Time (ms)", System.currentTimeMillis() - startTime);
        telemetry.update();
        sumOfLinearError += drivetrain.gyroSensor.getAngle() - degrees;

        sleep(5000);

    }
    private void turnQuadratic(int degrees, boolean right) {
        drivetrain.gyroSensor.zero();
        drivetrain.encoderSetup();
        long startTime = System.currentTimeMillis();

        telemetry.addLine("Linear to Proportion Completed");
        telemetry.addData("Turn Degrees: ", degrees);
        telemetry.update();

        int currAngle = Math.abs(drivetrain.gyroSensor.getAngle()); // Use getAngle() because it returns angle robot has turned from origin
        double pow; // power applied to motors
        while (currAngle < degrees && !isStopRequested()) {
            pow = quadraticToProportion((double) currAngle / degrees);
            if (right)
                drivetrain.setPowers(pow, -pow, 0);
            else
                drivetrain.setPowers(-pow, pow, 0);
            currAngle = Math.abs(drivetrain.gyroSensor.getAngle());
        }
        drivetrain.setPowers(0, 0, 0);

        telemetry.addData("Gyro Sensor Reading", drivetrain.gyroSensor.getAngle());
        telemetry.addData("Error (Degrees)", drivetrain.gyroSensor.getAngle() - degrees);
        telemetry.addData("Turn Time (ms)", System.currentTimeMillis() - startTime);
        telemetry.update();
        sumOfQuadraticError += drivetrain.gyroSensor.getAngle() - degrees;

        sleep(5000);
    }
    private void turnCubic(int degrees, boolean right) {
        drivetrain.gyroSensor.zero();
        drivetrain.encoderSetup();
        long startTime = System.currentTimeMillis();

        telemetry.addLine("Linear to Proportion Completed");
        telemetry.addData("Turn Degrees: ", degrees);
        telemetry.update();

        int currAngle = Math.abs(drivetrain.gyroSensor.getAngle()); // Use getAngle() because it returns angle robot has turned from origin
        double pow; // power applied to motors
        while (currAngle < degrees && !isStopRequested()) {
            pow = cubicToProportion((double) currAngle / degrees);
            if (right)
                drivetrain.setPowers(pow, -pow, 0);
            else
                drivetrain.setPowers(-pow, pow, 0);
            currAngle = Math.abs(drivetrain.gyroSensor.getAngle());
        }
        drivetrain.setPowers(0, 0, 0);

        telemetry.addData("Gyro Sensor Reading", drivetrain.gyroSensor.getAngle());
        telemetry.addData("Error (Degrees)", drivetrain.gyroSensor.getAngle() - degrees);
        telemetry.addData("Turn Time (ms)", System.currentTimeMillis() - startTime);
        telemetry.update();
        sumOfCubicError += drivetrain.gyroSensor.getAngle() - degrees;

        sleep(5000);
    }
    private void turnDegreesLeft(int degrees, boolean right) {
        drivetrain.gyroSensor.zero();
        drivetrain.encoderSetup();
        long startTime = System.currentTimeMillis();

        telemetry.addLine("Linear to Degrees Left");
        telemetry.addData("Turn Degrees: ", degrees);
        telemetry.update();

        int currAngle = Math.abs(drivetrain.gyroSensor.getAngle()); // Use getAngle() because it returns angle robot has turned from origin
        double pow; // power applied to motors
        while (currAngle < degrees && !isStopRequested()) {
            pow = linearToDegreesLeft(degrees - currAngle);
            if (right)
                drivetrain.setPowers(pow, -pow, 0);
            else
                drivetrain.setPowers(-pow, pow, 0);
            currAngle = Math.abs(drivetrain.gyroSensor.getAngle());
        }
        drivetrain.setPowers(0, 0, 0);

        telemetry.addData("Gyro Sensor Reading", drivetrain.gyroSensor.getAngle());
        telemetry.addData("Error (Degrees)", drivetrain.gyroSensor.getAngle() - degrees);
        telemetry.addData("Turn Time (ms)", System.currentTimeMillis() - startTime);
        telemetry.update();
        sumOfLinearDegreesError += drivetrain.gyroSensor.getAngle() - degrees;

        sleep(5000);
    }


    // RETURN THE POWER GIVEN A PROPORTION

    private double linearToDegreesLeft(int degreesLeft) {
        return START_POWER * (double) degreesLeft / 180;
    }
    private double linearToProportion(double proportion) {
        return -START_POWER * (proportion - 1);
    }
    private double quadraticToProportion(double proportion) {
        return START_POWER * Math.pow((proportion - 1), 2);
    }
    private double cubicToProportion(double proportion) {
        return -START_POWER * Math.pow((proportion - 1), 3);
    }
}
