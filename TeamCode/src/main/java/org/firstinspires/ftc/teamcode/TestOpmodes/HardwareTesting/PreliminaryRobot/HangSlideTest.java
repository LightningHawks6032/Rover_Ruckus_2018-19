package org.firstinspires.ftc.teamcode.TestOpmodes.HardwareTesting.PreliminaryRobot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.PrelimBot_Hardware;

// @TeleOp(name="Robot 1 Hanging Test", group="Iterative Opmode")
public class HangSlideTest extends OpMode {
    private PrelimBot_Hardware hardware;
    private double neverestPow = 0;

    public void init() {
        hardware = new PrelimBot_Hardware(hardwareMap, gamepad1, false);
        hardware.initHardware();
    }

    public void loop() {
        neverestPow = -gamepad1.right_stick_y;

        //hardware.hangVex.setPower(vexPow);
        hardware.hangNvst.setPower(neverestPow);

        //telemetry.addLine("Left joystick y = vex");
        telemetry.addLine("Right joystick y = nvst");
        //telemetry.addData("Hang Vex Power (Bring slide up)", hardware.hangVex.getPower());
        telemetry.addData("Hang Neverest Power (Bring slide down)", hardware.hangNvst.getPower());
        telemetry.update();
    }
}
