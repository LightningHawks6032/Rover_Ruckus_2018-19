package org.firstinspires.ftc.teamcode.TestOpmodes.HardwareTesting.Robot2;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Robot2_Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Robot2_Intake;

@TeleOp(name="Intake Flipper Test", group="Opmode")
public class IntakeFlipperTest extends OpMode {
    private Robot2_Hardware hardware;
    private Robot2_Intake intake;

    public void init() {
        hardware = new Robot2_Hardware(hardwareMap, gamepad1, gamepad2, false);
        hardware.initHardware();
        intake = hardware.intake;
    }

    public void loop() {
        // Will need to set constraints for power based on flipEncoder (adjust intake.FLIPPER_OUT_ENCODER_VAL accordingly)
        // I've set the power to be very low to avoid breaking anything
        intake.flipper.setPower(-gamepad2.left_stick_y * 0.2);

        telemetry.addData("Flipper Encoder Val", intake.flipEncoder.getEncoderCount());
        telemetry.addData("Flipper Motor Power", intake.flipper.getPower());
        telemetry.update();
    }

}