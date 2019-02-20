package org.firstinspires.ftc.teamcode.TestOpmodes.AutoTesting.StatesBot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.MecanumWheelDrive;
import org.firstinspires.ftc.teamcode.Hardware.OmniSlideDrive;
import org.firstinspires.ftc.teamcode.Hardware.QualBot_Hardware;
import org.firstinspires.ftc.teamcode.Hardware.StatesBot_Hardware;

public class TurningWithGyro extends LinearOpMode {
    private StatesBot_Hardware hardware;
    private MecanumWheelDrive drivetrain;

    public void runOpMode() {
        hardware = new StatesBot_Hardware(hardwareMap, gamepad1, gamepad2, true);
        hardware.initHardware();
        drivetrain = hardware.drivetrain;
        drivetrain.setAuto(this);
        waitForStart();
        drivetrain.setStartTime(System.currentTimeMillis());
        
        hardware.drivetrain.turn(90, true);
    }
}
