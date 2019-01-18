package org.firstinspires.ftc.teamcode.TestOpmodes.HardwareTesting.OfficialRobot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Hardware.OfficialBot_Hardware;

@TeleOp(name="IMU test", group="Test Opmode")
public class ExpansionHubIMUTest extends OpMode {
    private OfficialBot_Hardware hardware;
    private Orientation angles;

    public void init() {
        // hardware init (inits the gyro by calibrating and zeroing)
        hardware = new OfficialBot_Hardware(hardwareMap, gamepad1, gamepad2, true);
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
