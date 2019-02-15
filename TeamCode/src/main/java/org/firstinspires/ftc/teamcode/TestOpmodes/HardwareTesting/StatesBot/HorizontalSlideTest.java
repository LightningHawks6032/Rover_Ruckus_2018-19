package org.firstinspires.ftc.teamcode.TestOpmodes.HardwareTesting.StatesBot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.QualBot_Hardware;
import org.firstinspires.ftc.teamcode.Hardware.QualBot_Intake;
import org.firstinspires.ftc.teamcode.Hardware.StatesBot_Hardware;
import org.firstinspires.ftc.teamcode.Hardware.StatesBot_Intake;

@TeleOp(name="States Horizontal Slide Test", group="Test Opmode")
public class HorizontalSlideTest extends OpMode {
    private StatesBot_Hardware hardware;
    private StatesBot_Intake intake;

    public void init() {
        hardware = new StatesBot_Hardware(hardwareMap, gamepad1, gamepad2, false);
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
