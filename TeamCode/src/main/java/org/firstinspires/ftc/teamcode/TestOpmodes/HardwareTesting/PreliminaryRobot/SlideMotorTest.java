package org.firstinspires.ftc.teamcode.TestOpmodes.HardwareTesting.PreliminaryRobot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.PrelimBot_Hardware;

// @TeleOp(name="Robot1 Slide Test", group="Iterative Opmode")
public class SlideMotorTest extends OpMode {
    private PrelimBot_Hardware hardware;

    double motorPower;

    public void init() {
        //Initialize hardware
        hardware = new PrelimBot_Hardware(hardwareMap, gamepad1, false);
        hardware.initHardware(true);
    }

    public void loop() {
        motorPower = gamepad2.left_stick_y;


        // Update motor power
        hardware.slideMotor.setPower(motorPower);

        // Print to telemetry
        telemetry.addLine("Use left joy stick Y for motor power");
        telemetry.addData("Horizontal Slide Motor Power", hardware.slideMotor.getPower());
        telemetry.update();
    }
}
