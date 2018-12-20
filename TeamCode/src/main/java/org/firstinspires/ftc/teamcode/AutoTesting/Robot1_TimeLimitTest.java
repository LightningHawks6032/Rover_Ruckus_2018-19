package org.firstinspires.ftc.teamcode.AutoTesting;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Robot1_Hardware;

@Autonomous(name="Robot 1 Time Limit Test", group="Linear Opmode")
public class Robot1_TimeLimitTest extends LinearOpMode {

    private Robot1_Hardware hardware;
    private long startTime = System.currentTimeMillis();

    @Override
    public void runOpMode() {
        hardware = new Robot1_Hardware(hardwareMap, gamepad1, false);
        hardware.initHardware();

        turn(60, true);
    }

    public void driveDistance(int direction, double distance, double pow) {
        hardware.drivetrain.leftEncoder.reset();
        hardware.drivetrain.rightEncoder.reset();

        hardware.drivetrain.leftEncoder.runToPosition();
        hardware.drivetrain.rightEncoder.runToPosition();

        hardware.drivetrain.leftEncoder.setTarget(direction * distance);
        hardware.drivetrain.rightEncoder.setTarget(direction * distance);

        hardware.drivetrain.setPowers(direction * pow, direction * pow, 0);

        // This works --> make sure to add this to auto class
        while (hardware.drivetrain.leftMotor.isBusy() && hardware.drivetrain.rightMotor.isBusy() && System.currentTimeMillis() - startTime < 30000 && !isStopRequested()) {
            // WAIT - Motors are busy
        }

        hardware.drivetrain.setPowers(0, 0, 0);

        hardware.drivetrain.leftEncoder.runWithout();
        hardware.drivetrain.rightEncoder.runWithout();
    }

    public void turn(int degrees, boolean right) {
        hardware.drivetrain.gyroSensor.zero();
        hardware.drivetrain.encoderSetup();

        int currAngle = Math.abs(hardware.drivetrain.gyroSensor.getAngle()); // Use getAngle() because it returns angle robot has turned from origin
        double pow = 1; // power applied to motors

        while (currAngle < degrees && System.currentTimeMillis() - startTime < 30000 && !isStopRequested()) {
            pow = (double) (degrees - currAngle) / degrees * 0.6 + 0.1;

            // Apply power to motors and update currAngle
            if (right)
                hardware.drivetrain.setPowers(pow, -pow, 0);
            else
                hardware.drivetrain.setPowers(-pow, pow, 0);
            currAngle = Math.abs(hardware.drivetrain.gyroSensor.getAngle());
        }
        hardware.drivetrain.setPowers(0, 0, 0);
    }
}
