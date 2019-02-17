package org.firstinspires.ftc.teamcode.TestOpmodes.HardwareTesting.QualBot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.TestOpmodes.HardwareTesting.ServoTester;

@TeleOp(name="Qual Dumper Servo Test", group="Test Opmode")
public class DumperServoTest extends OpMode{
    private ServoTester testLeft;
    private ServoTester testRight;

    public void init() {
        testLeft = new ServoTester(this, hardwareMap.get(Servo.class, "lsv"), "Left Outtake", gamepad1);
        testRight = new ServoTester(this, hardwareMap.get(Servo.class, "rsv"), "Right Outtake", gamepad2);
    }

    public void loop() {
        testLeft.run();
        testRight.run();
        telemetry.update();
    }
}
