package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.teamcode.Robot1_Hardware;

@TeleOp(name="Mineral Detection Test", group="DogeCV")
public class MineralDetectionDrive extends MineralDetectionTest {
    Robot1_Hardware hardware;

    public void init() {

        hardware = new Robot1_Hardware(hardwareMap);
        hardware.initHardware();
        super.init();
    }

    public void loop() {
        super.loop();

        if (gamepad1.left_trigger > 0) {
            hardware.leftDrive.setPower(0);
            hardware.rightDrive.setPower(0);
            hardware.middleDrive.setPower(-0.8);
        } else if (gamepad1.right_trigger > 0) {
            hardware.leftDrive.setPower(0);
            hardware.rightDrive.setPower(0);
            hardware.middleDrive.setPower(0.8);
        } else {
            hardware.leftDrive.setPower(-gamepad1.left_stick_y);
            hardware.rightDrive.setPower(-gamepad1.right_stick_y);
            hardware.middleDrive.setPower(0);
        }
    }
}
