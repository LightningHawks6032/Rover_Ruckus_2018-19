package org.firstinspires.ftc.teamcode.HardwareTesting.Robot1;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Hardware.Robot1_Hardware;

// @TeleOp(name="Robot 1 Marker Arm Test", group="Iterative Opmode")
public class MarkerArmTest extends OpMode{
    Robot1_Hardware hardware;

    double servoPos;

    public void init() {
        // hardware init
        hardware = new Robot1_Hardware(hardwareMap, gamepad1, false);
        hardware.initHardware();

        servoPos = hardware.MARKER_ARM_UP;
    }

    public void loop() {
        hardware.drivetrain.manageTeleOp();
        testServo();
        telemetry.addData("Servo Position: ", hardware.markerArm.getPosition());
        telemetry.update();
    }

    private void testServo() {
        if(gamepad1.a)
            servoPos = 1;
        else if(gamepad1.b)
            servoPos = 0.5;
        else if(gamepad1.x)
            servoPos = 0;

        hardware.markerArm.setPosition(servoPos);
    }
}
