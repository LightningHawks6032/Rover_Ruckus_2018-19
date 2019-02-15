package org.firstinspires.ftc.teamcode.TestOpmodes.HardwareTesting.StatesBot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.QualBot_Hardware;
import org.firstinspires.ftc.teamcode.Hardware.StatesBot_Hardware;

@TeleOp(name="States Sensor Test", group="Test Opmode")
public class SensorTest extends OpMode {
    private StatesBot_Hardware hardware;

    public void init() {
        // hardware init (inits the gyro by calibrating and zeroing)
        hardware = new StatesBot_Hardware(hardwareMap, gamepad1, gamepad2, true);
        hardware.initHardware();
    }

    public void loop() {
        hardware.drivetrain.manageTeleOp();
        telemetry.addData("Gyro Heading", hardware.drivetrain.gyro.getHeading());
        telemetry.addData("Range Sensor Raw Ultrasonic", hardware.rangeSensor.rawUltrasonic());
        telemetry.addData("Range Sensor Raw Optical", hardware.rangeSensor.rawOptical());
        telemetry.addData("Range Sensor cm Optical", hardware.rangeSensor.cmOptical());
        telemetry.addData("Range Sensor Distance (inch)", hardware.rangeSensor.getDistance(DistanceUnit.INCH));
        telemetry.update();

        if (gamepad1.x) {
            hardware.drivetrain.gyro.zero();
        }
    }
}
