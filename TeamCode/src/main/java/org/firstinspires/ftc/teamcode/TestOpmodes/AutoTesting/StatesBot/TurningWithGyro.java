package org.firstinspires.ftc.teamcode.TestOpmodes.AutoTesting.StatesBot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutonomousData;
import org.firstinspires.ftc.teamcode.Hardware.MecanumWheelDrive;
import org.firstinspires.ftc.teamcode.Hardware.OmniSlideDrive;
import org.firstinspires.ftc.teamcode.Hardware.QualBot_Hardware;
import org.firstinspires.ftc.teamcode.Hardware.StatesBot_Hardware;

@Autonomous(name="Turning with Gyro", group= AutonomousData.TEST_GROUP)
public class TurningWithGyro extends LinearOpMode {
    private StatesBot_Hardware hardware;
    private MecanumWheelDrive drivetrain;

    public void runOpMode() {
        hardware = new StatesBot_Hardware(hardwareMap, gamepad1, gamepad2, true);
        hardware.initHardware();
        drivetrain = hardware.drivetrain;
        waitForStart();
        turn(90, true, 2);
        turnDegreesLeft(90, true);
        turn(20, true, 2);
        turnDegreesLeft(20, true);
    }

    public void turn(int degrees, boolean right, int polynomialDegree) {
        drivetrain.gyro.zero();
        drivetrain.encoderSetup();

        int currAngle = Math.abs(drivetrain.gyro.getAngle()); // Use getAngle() because it returns angle robot has turned from origin
        double startPow = 1;
        double pow; // power applied to motors
        double prop; // proportion of angle completed
        int powSign = polynomialDegree % 2 == 0 ? 1 : -1;

        telemetry.addData("Degrees to turn", degrees);
        telemetry.addData("Polynomial Degree", polynomialDegree);
        telemetry.update();

        while (currAngle < degrees) {
            prop = (double) currAngle / degrees;
            pow = powSign * startPow * Math.pow(prop - 1, polynomialDegree) + 0.1;

            // Apply power to motors and update currAngle
            if (right)
                drivetrain.setPowers(pow, -pow, pow, -pow);
            else
                drivetrain.setPowers(-pow, pow, -pow, pow);
            currAngle = Math.abs(drivetrain.gyro.getAngle());
        }
        drivetrain.setPowers(0, 0, 0, 0);

        telemetry.addData("Angle Turned", drivetrain.gyro.getAngle());
        telemetry.update();

        sleep(10000);
    }

    public void turnDegreesLeft(int degrees, boolean right) {
        drivetrain.gyro.zero();
        drivetrain.encoderSetup();

        int currAngle = Math.abs(drivetrain.gyro.getAngle()); // Use getAngle() because it returns angle robot has turned from origin
        double startPow = 1;
        double pow; // power applied to motors
        double prop; // proportion of angle completed

        telemetry.addData("Degrees to turn w/ degrees left", degrees);
        telemetry.update();

        while (currAngle < degrees) {
            prop = (double) (degrees - currAngle) / 90;
            pow = startPow * prop;

            // Apply power to motors and update currAngle
            if (right)
                drivetrain.setPowers(pow, -pow, pow, -pow);
            else
                drivetrain.setPowers(-pow, pow, -pow, pow);
            currAngle = Math.abs(drivetrain.gyro.getAngle());
        }
        drivetrain.setPowers(0, 0, 0, 0);

        telemetry.addData("Angle Turned", drivetrain.gyro.getAngle());
        telemetry.update();

        sleep(10000);
    }
}
