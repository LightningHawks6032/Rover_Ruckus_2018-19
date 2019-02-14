package org.firstinspires.ftc.teamcode.TestOpmodes.HardwareTesting.StatesBot;

import com.qualcomm.hardware.motors.RevRobotics20HdHexMotor;
import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Hex Motor test", group="Test Opmode")
public class HexMotorTest extends OpMode {
    private DcMotor motor;

    public void init() {
        motor = hardwareMap.get(DcMotor.class, "hm");
    }

    public void loop() {
        motor.setPower(-gamepad1.left_stick_y);
        telemetry.addLine("Use left stick y for power");
        telemetry.addData("Motor power", motor.getPower());
        telemetry.update();
    }
}