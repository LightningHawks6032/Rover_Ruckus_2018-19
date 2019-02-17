package org.firstinspires.ftc.teamcode.TestOpmodes.HardwareTesting.QualBot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.QualBot_Hardware;

@TeleOp(name="Qual Sensor Test", group="Test Opmode")
public class SensorTest extends OpMode {
    private QualBot_Hardware hardware;

    public void init() {
        // hardware init (inits the gyro by calibrating and zeroing)
        hardware = new QualBot_Hardware(hardwareMap, gamepad1, gamepad2, true);
        hardware.initHardware();
    }

    public void loop() {
        hardware.drivetrain.manageTeleOp();
        telemetry.addData("Gyro Heading", hardware.drivetrain.getGyro().getHeading());
        telemetry.addData("Range Sensor Raw Ultrasonic", hardware.rangeSensor.rawUltrasonic());
        telemetry.addData("Range Sensor Raw Optical", hardware.rangeSensor.rawOptical());
        telemetry.addData("Range Sensor cm Optical", hardware.rangeSensor.cmOptical());
        telemetry.addData("Range Sensor Distance (inch)", hardware.rangeSensor.getDistance(DistanceUnit.INCH));
        telemetry.update();

        if (gamepad1.x) {
            hardware.drivetrain.getGyro().zero();
        }
    }
}
