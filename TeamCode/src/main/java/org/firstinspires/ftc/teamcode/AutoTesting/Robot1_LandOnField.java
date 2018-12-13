package org.firstinspires.ftc.teamcode.AutoTesting;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FieldMapping.FieldMap;
import org.firstinspires.ftc.teamcode.Hardware.Robot1_Hardware;
import org.firstinspires.ftc.teamcode.Vision.Detectors.GoldAlignDetector;

@Autonomous(name="Landing", group="Linear OpMode")
public class Robot1_LandOnField extends LinearOpMode {
    // Declare hardware
    private Robot1_Hardware hardware;

    public void runOpMode() throws InterruptedException {
        hardware = new Robot1_Hardware(hardwareMap, gamepad1, true);
        hardware.initHardware();

        waitForStart();
        land();
        hardware.drivetrain.strafeDistance(-1, 5, 0.5);
        hardware.drivetrain.driveDistance(1, 5, 0.5);
        hardware.drivetrain.strafeDistance(1, 5, 0.5);
        hardware.drivetrain.driveDistance(1, 5, 0.5);
        //hardware.drivetrain.strafeForTime(-0.5, 1);

    }

    private void land() {
        hardware.hangEncoder.reset();

        hardware.hangEncoder.runToPosition();

        hardware.hangEncoder.setEncoderTarget(16500);

        hardware.hangNvst.setPower(-1);

        while (hardware.hangNvst.isBusy()) {
            // WAIT - Motor is busy
        }

        hardware.hangNvst.setPower(0);
    }
}