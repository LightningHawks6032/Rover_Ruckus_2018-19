package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class StatesBot_Hardware {
    // Declaring the hardware components
    public MecanumWheelDrive drivetrain;


    public StatesBot_Hardware(HardwareMap hardwareMap, Gamepad driveGamepad, Gamepad manipsGamepad, boolean calibrateSensors) {
        drivetrain = new MecanumWheelDrive(
                hardwareMap.get(DcMotor.class, "lfront"),
                hardwareMap.get(DcMotor.class, "rfront"),
                hardwareMap.get(DcMotor.class, "lback"),
                hardwareMap.get(DcMotor.class, "rback"),
                new ExpansionHubIMU(hardwareMap.get(BNO055IMU.class, "imu"), calibrateSensors),
                new MRGyro(hardwareMap.get(GyroSensor.class, "gs"), calibrateSensors),
                driveGamepad
        );
    }
}
