package org.firstinspires.ftc.teamcode.TestOpmodes.HardwareTesting;

import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Hex Motor test", group="Test Opmode")
public class HexMotorTest extends OpMode {
    private RevRoboticsCoreHexMotor motor;

    public void init() {
        motor = hardwareMap.get(RevRoboticsCoreHexMotor.class, "hm");

    }

    public void loop() {

    }
}