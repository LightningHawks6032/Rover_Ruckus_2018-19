package org.firstinspires.ftc.teamcode.TestOpmodes.HardwareTesting.StatesBot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.TestOpmodes.HardwareTesting.ServoTester;

@TeleOp(name="States Intake servo test", group="Test Opmode")
public class IntakeServoTest extends OpMode {
    private ServoTester flipperLeft;
    private ServoTester flipperRight;

    public void init() {
        flipperLeft = new ServoTester(this, hardwareMap.get(Servo.class, "flip"), "Left Intake Flipper", gamepad1);
        flipperRight = new ServoTester(this, hardwareMap.get(Servo.class, "flip"), "Right Intake Flipper", gamepad2);
    }

    public void loop() {
        flipperLeft.run();
        flipperRight.run();
        telemetry.update();
    }
}