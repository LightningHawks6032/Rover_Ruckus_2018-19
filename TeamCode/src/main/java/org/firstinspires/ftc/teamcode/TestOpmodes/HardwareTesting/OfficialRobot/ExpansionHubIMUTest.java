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

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        hardware.imu.initialize(parameters);
    }

    public void loop() {
        angles = hardware.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        hardware.drivetrain.manageTeleOp();
        telemetry.addData("IMU Pitch (x-axis)", angles.thirdAngle);
        telemetry.addData("IMU Roll (y-axis)", angles.secondAngle);
        telemetry.addData("IMU Heading/Yaw (z-axis)", angles.firstAngle);
        telemetry.update();
    }
}
