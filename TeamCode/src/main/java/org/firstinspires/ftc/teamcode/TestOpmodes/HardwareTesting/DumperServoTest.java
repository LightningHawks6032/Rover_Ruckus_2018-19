package org.firstinspires.ftc.teamcode.TestOpmodes.HardwareTesting;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.OfficialBot_Hardware;
import org.firstinspires.ftc.teamcode.Hardware.OfficialBot_Intake;
import org.firstinspires.ftc.teamcode.Hardware.OfficialBot_Outtake;
import org.firstinspires.ftc.teamcode.TestOpmodes.ServoTester;

@TeleOp(name="Dumper Servo Test", group="Test Opmode")
public class DumperServoTest extends OpMode{
    private ServoTester testLeft;
    private ServoTester testRight;

    public void init() {
        testLeft = new ServoTester(this, hardwareMap.get(Servo.class, "lsv"), gamepad1);
        testRight = new ServoTester(this, hardwareMap.get(Servo.class, "rsv"), gamepad2);
    }

    public void loop() {
        testLeft.run();
        testRight.run();
    }
}
