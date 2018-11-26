package org.firstinspires.ftc.teamcode.HardwareTesting;

import org.firstinspires.ftc.teamcode.Hardware.Encoder;
import org.firstinspires.ftc.teamcode.Hardware.Robot1_Hardware;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;


@TeleOp(name="Robot 1 Gyro Test", group="Iterative Opmode")
public class Robot1_GyroTest extends OpMode {
    Robot1_Hardware hardware;

    public void init(){
        // hardware init
        hardware = new Robot1_Hardware(hardwareMap, gamepad1);
        hardware.initHardware();

        hardware.gyroSensor.resetZAxisIntegrator();
        calibrateGyro();
    }

    public void loop(){
        hardware.drivetrain.manageTeleOp();
        telemetry.addData("Gyro X :", hardware.gyroSensor.rawX());
        telemetry.addData("Gyro Y: ", hardware.gyroSensor.rawY());
        telemetry.addData("Gyro Z: ", hardware.gyroSensor.rawZ());
        telemetry.addData("Cartesian Z Heading: ", hardware.gyroSensor.getHeading());
        telemetry.update();

        if (gamepad1.x)
            hardware.gyroSensor.resetZAxisIntegrator();
    }

    private void calibrateGyro() {
        hardware.gyroSensor.calibrate();
        while(hardware.gyroSensor.isCalibrating()) {
            // WAIT - Gyro Sensor is Calibrating
        }
    }
}
