package org.firstinspires.ftc.teamcode.TestOpmodes.HardwareTesting.StatesBot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Hardware.QualBot_Hardware;
import org.firstinspires.ftc.teamcode.Hardware.StatesBot_Hardware;

@TeleOp(name="States IMU test", group="Test Opmode")
public class IMUTest extends OpMode {
    private StatesBot_Hardware hardware;

    public void init() {
        // hardware init (inits the gyro by calibrating and zeroing)
        hardware = new StatesBot_Hardware(hardwareMap, gamepad1, gamepad2, true);
        hardware.initHardware();
    }

    public void loop() {
        hardware.drivetrain.manageTeleOp();
        telemetry.addData("IMU Pitch (x-axis)", hardware.drivetrain.imu.getAngles()[0]);
        telemetry.addData("IMU Roll (y-axis)", hardware.drivetrain.imu.getAngles()[1]);
        telemetry.addData("IMU Heading/Yaw (z-axis)", hardware.drivetrain.imu.getHeading());
        telemetry.update();
    }
}
