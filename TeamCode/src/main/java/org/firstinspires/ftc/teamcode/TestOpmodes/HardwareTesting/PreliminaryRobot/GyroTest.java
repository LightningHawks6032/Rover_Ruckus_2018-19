package org.firstinspires.ftc.teamcode.TestOpmodes.HardwareTesting.PreliminaryRobot;

import org.firstinspires.ftc.teamcode.Hardware.Encoder;
import org.firstinspires.ftc.teamcode.Hardware.PrelimBot_Hardware;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;


// @TeleOp(name="Robot 1 Gyro Test", group="Iterative Opmode")
public class GyroTest extends OpMode {
    PrelimBot_Hardware hardware;

    public void init(){
        // hardware init (inits the gyro by calibrating and zeroing)
        hardware = new PrelimBot_Hardware(hardwareMap, gamepad1, true);
        hardware.initHardware();
    }

    public void loop() {
        hardware.drivetrain.manageTeleOp();
        //telemetry.addData("Gyro X :", hardware.gyroSensor.rawX());
        //telemetry.addData("Gyro Y: ", hardware.gyroSensor.rawY());
        //telemetry.addData("Gyro Z: ", hardware.gyroSensor.rawZ());
        telemetry.addData("Cartesian Z Heading: ", hardware.drivetrain.getGyro().getHeading());
        telemetry.addData("getAngle()", hardware.drivetrain.getGyro().getAngle());
        telemetry.update();

        if (gamepad1.x) {
            hardware.drivetrain.getGyro().zero();
        }
    }
}
