package org.firstinspires.ftc.teamcode.TestOpmodes.HardwareTesting.StatesBot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.TestOpmodes.HardwareTesting.ServoTester;

@TeleOp(name="States Outtake servo test", group="Test Opmode")
public class OuttakeServoTest extends OpMode {
    private ServoTester leftServo;
    private ServoTester rightServo;

    public void init() {
        leftServo = new ServoTester(this, hardwareMap.get(Servo.class, "lsv"), "Left Outtake Servo", gamepad1);
        rightServo = new ServoTester(this, hardwareMap.get(Servo.class, "rsv"), "Right Outtake Servo", gamepad2);
    }

    public void loop() {
        leftServo.run();
        rightServo.run();
        telemetry.update();
    }
}