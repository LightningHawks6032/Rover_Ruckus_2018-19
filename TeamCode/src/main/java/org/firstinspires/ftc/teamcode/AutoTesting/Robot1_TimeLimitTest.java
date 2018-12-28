package org.firstinspires.ftc.teamcode.AutoTesting;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutonomousData;
import org.firstinspires.ftc.teamcode.Hardware.Robot1_Hardware;
import org.firstinspires.ftc.teamcode.Robot1.Robot1_Auto;

@Autonomous(name="Robot 1 Time Limit Test", group=AutonomousData.TEST_GROUP)
public class Robot1_TimeLimitTest extends LinearOpMode {

    private Robot1_Hardware hardware;
    private Robot1_Auto auto;

    @Override
    public void runOpMode() {
        hardware = new Robot1_Hardware(hardwareMap, gamepad1, false);
        hardware.initHardware();
        auto = new Robot1_Auto(this, hardware);

        waitForStart();
        auto.setStartTime(System.currentTimeMillis()); // needed at beginning of auto

        hardware.drivetrain.turn(60, true);
    }

    /*
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
    */
}
