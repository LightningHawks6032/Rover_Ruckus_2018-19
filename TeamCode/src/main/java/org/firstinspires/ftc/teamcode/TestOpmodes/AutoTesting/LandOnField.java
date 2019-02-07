package org.firstinspires.ftc.teamcode.TestOpmodes.AutoTesting;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.QualBot_Hardware;
import org.firstinspires.ftc.teamcode.QualifierRobot.Auto;

// @Autonomous(name="Landing", group= AutonomousData.TEST_GROUP)
public class LandOnField extends LinearOpMode {
    private QualBot_Hardware hardware;
    private Auto auto;

    public void runOpMode() throws InterruptedException {
        hardware = new QualBot_Hardware(hardwareMap, gamepad1, gamepad2, true);
        auto = new Auto(this, hardware);
        hardware.initHardware();

        waitForStart();
        auto.setStartTime(System.currentTimeMillis());

        hardware.outtake.landOnField();
    }
}
