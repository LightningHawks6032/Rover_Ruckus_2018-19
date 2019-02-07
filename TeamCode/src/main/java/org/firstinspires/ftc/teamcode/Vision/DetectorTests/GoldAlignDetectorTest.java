package org.firstinspires.ftc.teamcode.Vision.DetectorTests;

import org.firstinspires.ftc.teamcode.Hardware.QualBot_Hardware;
import org.firstinspires.ftc.teamcode.Vision.Detectors.GoldAlignDetector;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="GoldAlign Detector", group="DogeCV")
public class GoldAlignDetectorTest extends OpMode
{
    // Detector object
    private GoldAlignDetector detector;
    private QualBot_Hardware hardware;


    @Override
    public void init() {
        // Set up detector
        hardware = new QualBot_Hardware(hardwareMap, gamepad1, gamepad2, false);
        hardware.initHardware();
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

        telemetry.addData("IsAligned" , detector.getAligned()); // Is the bot aligned with the gold mineral?
        telemetry.addData("X Pos" , detector.getXPosition()); // Gold X position.
        telemetry.addData("Y Pos", detector.getYPosition()); // Gold Y position.
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

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        // Disable the detector
        detector.disable();
    }

}