package org.firstinspires.ftc.teamcode.TestOpmodes.HardwareTesting.StatesBot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.TestOpmodes.HardwareTesting.ServoTester;

@TeleOp(name="States Intake servo test", group="Test Opmode")
public class IntakeServoTest extends OpMode {
    private ServoTester flipper;

    public void init() {
        flipper = new ServoTester(this, hardwareMap.get(Servo.class, "flip"), "Intake Flipper", gamepad1);
    }

    public void loop() {
        flipper.run();
        telemetry.update();
    }
}