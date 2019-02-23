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



    public void runOpMode() throws InterruptedException {
        hardware = new StatesBot_Hardware(hardwareMap, gamepad1, gamepad2, true);
        auto = new Auto(this, hardware);
        hardware.initHardware();

        Vector zero = new Vector(0,0);
        hardware.drivetrain.setRobotPos(zero);

        Vector initPos = hardware.drivetrain.robotPos;
        Vector targetPos = new Vector(initPos.getX() + 30, initPos.getY() + 30);
        double leftLimit = initPos.getX() + 21;
        double rightLimit = initPos.getX() + 39;
        double targetAngle = 45;

        waitForStart();
        auto.setStartTime(System.currentTimeMillis());
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
    }
}
