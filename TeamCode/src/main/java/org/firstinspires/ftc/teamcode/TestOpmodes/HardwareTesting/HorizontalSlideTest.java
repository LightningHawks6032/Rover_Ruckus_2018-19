package org.firstinspires.ftc.teamcode.TestOpmodes.HardwareTesting;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.OfficialBot_Hardware;
import org.firstinspires.ftc.teamcode.Hardware.OfficialBot_Intake;
import org.firstinspires.ftc.teamcode.Hardware.OfficialBot_Outtake;

@TeleOp(name="Horizontal Slide Test", group="Test Opmode")
public class HorizontalSlideTest extends OpMode {
    private OfficialBot_Hardware hardware;
    private OfficialBot_Intake intake;

    public void init() {
        hardware = new OfficialBot_Hardware(hardwareMap, gamepad1, gamepad2, false);
        hardware.intake.slideEncoder.reset();
        hardware.initHardware();
        intake = hardware.intake;
    }

    public void loop() {
        double pow = -gamepad1.left_stick_y;
        intake.horizontalSlide.setPower(pow);

        telemetry.addLine("Use left stick on gamepad1");
        telemetry.addData("Horizontal Slide Pow", intake.horizontalSlide.getPower());
        telemetry.addData("Horizontal Slide Encoder", intake.slideEncoder.getEncoderCount());
        telemetry.update();

        if (gamepad1.x)
            intake.slideEncoder.setup();
    }
}
