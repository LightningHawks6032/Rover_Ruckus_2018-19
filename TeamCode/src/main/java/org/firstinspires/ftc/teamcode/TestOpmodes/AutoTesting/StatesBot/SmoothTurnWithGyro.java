package org.firstinspires.ftc.teamcode.TestOpmodes.AutoTesting.StatesBot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutonomousData;
import org.firstinspires.ftc.teamcode.FieldMapping.Vector;
import org.firstinspires.ftc.teamcode.Hardware.StatesBot_Hardware;
import org.firstinspires.ftc.teamcode.StatesRobot.Auto;

@Autonomous(name="Smooth Turn Test", group= AutonomousData.TEST_GROUP)

public class SmoothTurnWithGyro extends LinearOpMode {

    private StatesBot_Hardware hardware;
    private Auto auto;

    private double PERCENT_LIMIT = 0.6;
    private double PERCENT_MIN = 0.3;

    public void runOpMode() throws InterruptedException {
        hardware = new StatesBot_Hardware(hardwareMap, gamepad1, gamepad2, true);
        auto = new Auto(this, hardware);
        hardware.initHardware();

        waitForStart();
        auto.setStartTime(System.currentTimeMillis());

        /*

        Vector zero = new Vector(0,0);
        hardware.drivetrain.setRobotPos(zero);

        Vector initPos = hardware.drivetrain.robotPos;
        Vector targetPos = new Vector(initPos.getX() + 30, initPos.getY() + 30);
        double leftLimit = initPos.getX() + 21;
        double rightLimit = initPos.getX() + 39;
        double targetAngle = 45;
        */

        hardware.drivetrain.gyro.zero();
        hardware.drivetrain.encoderSetup();

        int currentAngle = Math.abs(hardware.drivetrain.gyro.getAngle());
        int targetAngle = 45;
        double leftPow = 1;
        double rightPow = 1;

        // amount between base power and full power
        double remainingPow = 1 - leftPow;
        double percentFromCompletion = 100;
        double percentCompleted = 0;
        double modFromCompletion = PERCENT_LIMIT;
        double powMod;

        waitForStart();
        auto.setStartTime(System.currentTimeMillis());

        /*
        while (hardware.drivetrain.robotPos.getY() < targetPos.getY()) {
            double leftPow = 1;
            double rightPow = 1;
            double angleOffset = targetAngle - hardware.drivetrain.gyro.getAngle();
            double powRatio = 1- (Math.abs(angleOffset) / 360);
            if (hardware.drivetrain.robotPos.getX() >= leftLimit && angleOffset > 2) {
                leftPow = rightPow * powRatio;
            }
            hardware.drivetrain.setPowers(leftPow, rightPow, leftPow, rightPow);
        }

        */

        // apply extra power to side on outside of turn
        // applied power should be proportional to percentage of turn yet to be completed
        // time/distance taken for turn should be related to leftPow

        while (currentAngle < targetAngle && auto.autoRunning()){
            percentCompleted = currentAngle/targetAngle;
            percentFromCompletion = 1 - percentCompleted;

            modFromCompletion = percentCompleted;
            if(modFromCompletion > PERCENT_LIMIT) modFromCompletion = PERCENT_LIMIT;
            if(modFromCompletion < PERCENT_LIMIT) modFromCompletion = PERCENT_MIN;


            powMod = (percentFromCompletion+0.01)*0.7;

            leftPow = rightPow*modFromCompletion;

            //if(hardware.drivetrain.leftFrontEncoder.getEncoderCount() > )

            hardware.drivetrain.setPowers(leftPow * powMod, rightPow * powMod, leftPow *powMod, rightPow * powMod);
            currentAngle = Math.abs(hardware.drivetrain.gyro.getAngle());

            telemetry.addData("Pow Ratio: ", rightPow/leftPow);
            telemetry.addData("currentAngle: ", currentAngle);
            telemetry.update();
        }
        hardware.drivetrain.setPowers(0,0,0,0);
    }

}
