package org.firstinspires.ftc.teamcode.TestOpmodes.HardwareTesting.PreliminaryRobot;

import org.firstinspires.ftc.teamcode.Hardware.Encoder;
import org.firstinspires.ftc.teamcode.Hardware.PrelimBot_Hardware;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

// @TeleOp(name="Robot 1 Encoder Test", group="Iterative Opmode")
public class EncoderTest extends OpMode {
    PrelimBot_Hardware hardware;

    Encoder leftDriveEncoder;
    Encoder rightDriveEncoder;

    public void init() {
        //Initialize Hardware
        hardware = new PrelimBot_Hardware(hardwareMap, gamepad1, false);
        hardware.initHardware();

        leftDriveEncoder = hardware.drivetrain.getLeftEncoder();
        rightDriveEncoder = hardware.drivetrain.getRightEncoder();
    }

    public void loop() {
        hardware.drivetrain.manageTeleOp();

        // debugging
        double avg = (leftDriveEncoder.linDistance() + rightDriveEncoder.linDistance())/2;
        telemetry.addData("Left Encoder count: ", leftDriveEncoder.getEncoderCount());
        telemetry.addData("Left Motor rotations: ", leftDriveEncoder.motorRotations());
        telemetry.addData("Right Encoder count: ", rightDriveEncoder.getEncoderCount());
        telemetry.addData("Right Motor rotations: ", rightDriveEncoder.motorRotations());
        telemetry.addData("Distance: ", avg);
        telemetry.update();

    }
}
