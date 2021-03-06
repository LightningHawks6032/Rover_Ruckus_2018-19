package org.firstinspires.ftc.teamcode.TestOpmodes.HardwareTesting.QualBot;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.QualBot_Hardware;
import org.firstinspires.ftc.teamcode.Hardware.QualBot_Intake;

// @TeleOp(name="Qual Intake Flipper Test", group="Test Opmode")
public class IntakeFlipperTest extends OpMode {
    private QualBot_Hardware hardware;
    private QualBot_Intake intake;

    public void init() {
        hardware = new QualBot_Hardware(hardwareMap, gamepad1, gamepad2, false);
        hardware.intake.flipEncoder.reset();
        hardware.initHardware();
        intake = hardware.intake;
    }

    public void loop() {
        // Will need to set constraints for power based on flipEncoder (adjust intake.FLIPPER_OUT_ENCODER_VAL accordingly)
        // I've set the power to be very low to avoid breaking anything
        intake.flipper.setPower(-gamepad1.left_stick_y * 0.4);
        //intake.manageTeleOp();
        telemetry.addData("Flipper Encoder Val", intake.flipEncoder.getEncoderCount());
        telemetry.addData("Flipper Motor Power", intake.flipper.getPower());
        telemetry.update();

        if (gamepad1.x)
            intake.flipEncoder.setup();
    }

}