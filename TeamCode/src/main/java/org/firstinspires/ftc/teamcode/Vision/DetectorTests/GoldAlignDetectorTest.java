package org.firstinspires.ftc.teamcode.Vision.DetectorTests;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;

import org.firstinspires.ftc.teamcode.Hardware.Robot2_Hardware;
import org.firstinspires.ftc.teamcode.Vision.Detectors.GoldAlignDetector;
import org.firstinspires.ftc.teamcode.Vision.Detectors.NavTargetDetector;

import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="GoldAlign Detector", group="DogeCV")
public class GoldAlignDetectorTest extends OpMode
{
    // Detector object
    private GoldAlignDetector detector;
    private Robot2_Hardware hardware;


    @Override
    public void init() {
        // Set up detector
        detector = hardware.mineralDetector; // Create detector
        detector.setupDetector(hardwareMap, 1); // Camera Index: 0 for back camera, 1 for front camera
    }

    /*
     * Code to run REPEATEDLY when the driver hits INIT
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

    }

    /*
     * Code to run REPEATEDLY when the driver hits PLAY
     */
    @Override
    public void loop() {
        hardware.drivetrain.manageTeleOp();
        testPhoneServo();

        telemetry.addData("Phone Servo Position", hardware.phoneServo.getPosition());
        telemetry.addData("IsAligned" , detector.getAligned()); // Is the bot aligned with the gold mineral?
        telemetry.addData("X Pos" , detector.getXPosition()); // Gold X position.
        telemetry.addData("Is found?", detector.isFound()); // Gold mineral found?
        if (detector.getXPosition() > detector.getRobotCenterX()) {
            telemetry.addLine("Robot Center X is less than Mineral Position ");
        } else if (detector.getXPosition() < detector.getRobotCenterX()) {
            telemetry.addLine("Robot Center X is greater than Mineral Position ");
        }
        telemetry.addData("Robot Center X", detector.getRobotCenterX());
        telemetry.addData("Gold Mineral Location", detector.mineralLocation());
        telemetry.update();
    }

    private void testPhoneServo() {
        if (gamepad1.dpad_up)
            hardware.phoneServo.setPosition(0);
        else if (gamepad1.dpad_right)
            hardware.phoneServo.setPosition(0.25);
        else if (gamepad1.dpad_down)
            hardware.phoneServo.setPosition(0.5);
        else if (gamepad1.dpad_left)
            hardware.phoneServo.setPosition(0.75);
        else if (gamepad1.x)
            hardware.phoneServo.setPosition(1);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        // Disable the detector
        detector.disable();
    }

}