package org.firstinspires.ftc.teamcode.TestOpmodes.HardwareTesting.Robot2;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Robot2_Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Robot2_Outtake;

@TeleOp(name="Vertical Slide Test", group="Opmode")
public class VerticalSlideTest extends OpMode{
    private Robot2_Hardware hardware;
    private Robot2_Outtake outtake;

    public void init(){
        hardware = new Robot2_Hardware(hardwareMap, gamepad1, gamepad2, false);
        hardware.initHardware();
        outtake = hardware.outtake;
    }

    public void loop(){
        double pow = -gamepad1.left_stick_y;

        outtake.leftVertical.setPower(pow);
        outtake.rightVertical.setPower(pow);

        telemetry.addLine("Use left stick on gamepad1");
        telemetry.addData("Left Pow", outtake.leftVertical.getPower());
        telemetry.addData("Right Pow", outtake.rightVertical.getPower());
        telemetry.addData("Left Encoder", outtake.leftVertEncoder.getEncoderCount());
        telemetry.addData("Right Encoder", outtake.rightVertEncoder.getEncoderCount());
        telemetry.update();
    }
}
