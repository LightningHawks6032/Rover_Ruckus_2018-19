package org.firstinspires.ftc.teamcode.TestOpmodes.HardwareTesting.StatesBot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.QualBot_Hardware;
import org.firstinspires.ftc.teamcode.Hardware.QualBot_Outtake;
import org.firstinspires.ftc.teamcode.Hardware.StatesBot_Hardware;
import org.firstinspires.ftc.teamcode.Hardware.StatesBot_Outtake;

@TeleOp(name="States Vertical Slide Test", group="Test Opmode")
public class VerticalSlideTest extends OpMode {
    private StatesBot_Hardware hardware;
    private StatesBot_Outtake outtake;

    public void init() {
        hardware = new StatesBot_Hardware(hardwareMap, gamepad1, gamepad2, false);
        hardware.outtake.leftVertEncoder.reset();
        hardware.outtake.rightVertEncoder.reset();
        hardware.initHardware();
        outtake = hardware.outtake;
    }

    public void loop() {
        double pow = -gamepad1.left_stick_y;

        outtake.leftVertical.setPower(pow);
        outtake.rightVertical.setPower(pow);

        telemetry.addLine("Use left stick on gamepad1");
        telemetry.addLine("Press x on gamepad1 to reset encoder");
        telemetry.addData("Left Pow", outtake.leftVertical.getPower());
        telemetry.addData("Right Pow", outtake.rightVertical.getPower());
        telemetry.addData("Left Encoder", outtake.leftVertEncoder.getEncoderCount());
        telemetry.addData("Right Encoder", outtake.rightVertEncoder.getEncoderCount());
        telemetry.update();

        if (gamepad1.x) {
            outtake.leftVertEncoder.setup();
            outtake.rightVertEncoder.setup();
        }
    }
}
