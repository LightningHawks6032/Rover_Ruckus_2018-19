package org.firstinspires.ftc.teamcode.TestOpmodes.HardwareTesting.Robot2;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.Robot1_Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Robot2_Hardware;

@TeleOp(name="MR Sensor Test", group="Iterative Opmode")
public class MRSensorTest extends OpMode {
    Robot2_Hardware hardware;

    public void init(){
        // hardware init (inits the gyro by calibrating and zeroing)
        hardware = new Robot2_Hardware(hardwareMap, gamepad1, gamepad2, false);
        hardware.initHardware();
    }

    public void loop() {
        hardware.drivetrain.manageTeleOp();
        telemetry.addData("Gyro Heading", hardware.drivetrain.getGyro().getHeading());
        telemetry.addData("Range Sensor Ultrasonic", hardware.rangeSensor.rawUltrasonic());
        telemetry.addData("Range Sensor Distance (inch)", hardware.rangeSensor.getDistance(DistanceUnit.INCH));
        telemetry.update();

        if (gamepad1.x) {
            hardware.drivetrain.getGyro().zero();
        }
    }
}
