package org.firstinspires.ftc.teamcode.TestOpmodes.AutoTesting.PreliminaryRobot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FieldMapping.FieldMap;
import org.firstinspires.ftc.teamcode.Hardware.PrelimBot_Hardware;
import org.firstinspires.ftc.teamcode.Vision.Detectors.GoldAlignDetector;

// @Autonomous(name="Landing", group=AutonomousData.TEST_GROUP)
public class LandOnField extends LinearOpMode {
    // Declare hardware
    private PrelimBot_Hardware hardware;

    public void runOpMode() throws InterruptedException {
        hardware = new PrelimBot_Hardware(hardwareMap, gamepad1, true);
        hardware.initHardware(true);

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