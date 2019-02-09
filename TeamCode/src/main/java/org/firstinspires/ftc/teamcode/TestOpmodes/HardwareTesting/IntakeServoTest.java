package org.firstinspires.ftc.teamcode.TestOpmodes.HardwareTesting;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="intake servo test", group="Test Opmode")
public class IntakeServoTest extends OpMode {
    private ServoTester test;

    public void init() {
        test = new ServoTester(this, hardwareMap.get(Servo.class, "ma"), gamepad1);
    }

    public void loop() {
        test.run();
        telemetry.update();
    }
}