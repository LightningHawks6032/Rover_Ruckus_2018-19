package org.firstinspires.ftc.teamcode.TestOpmodes.AutoTesting.StatesBot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutonomousData;
import org.firstinspires.ftc.teamcode.Hardware.StatesBot_Hardware;
import org.firstinspires.ftc.teamcode.StatesRobot.Auto;

@Autonomous(name="Landing", group= AutonomousData.TEST_GROUP)
public class LandOnField extends LinearOpMode {
    private StatesBot_Hardware hardware;
    private Auto auto;

    public void runOpMode() throws InterruptedException {
        hardware = new StatesBot_Hardware(hardwareMap, gamepad1, gamepad2, true);
        auto = new Auto(this, hardware);
        hardware.initHardware();

        waitForStart();
        auto.setStartTime(System.currentTimeMillis());

        hardware.outtake.verticalSlideUp();
    }
}
