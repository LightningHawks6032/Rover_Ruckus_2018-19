package org.firstinspires.ftc.teamcode.HardwareTesting;

import org.firstinspires.ftc.teamcode.Hardware.Encoder;
import org.firstinspires.ftc.teamcode.Hardware.Robot1_Hardware;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Robot1_EncoderTest extends OpMode {
    Robot1_Hardware hardware;

    Encoder encoder = new Encoder();

    public void init() {
        //Initialize Hardware
        hardware = new Robot1_Hardware(hardwareMap);
        hardware.initHardware();
    }

    public void loop() {
        telemetry.addData("Encoder count: ", encoder.getEncoderCount());
        telemetry.addData("Motor rotations: ", encoder.motorRotations());
        telemetry.addData("Distance: ", encoder.linDistance());
    }
}
