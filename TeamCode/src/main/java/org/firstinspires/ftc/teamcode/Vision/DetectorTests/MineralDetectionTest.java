package org.firstinspires.ftc.teamcode.Vision.DetectorTests;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Vision.Detectors.MineralDetector;


// @TeleOp(name="Mineral Detection Test", group="DogeCV")
public class MineralDetectionTest extends OpMode
{
    // Detector object
    private MineralDetector detector;


    @Override
    public void init() {
        // Set up detector
        detector = new MineralDetector("gold", true); // Create detector
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

    @Override
    public void loop() {
        telemetry.addLine("We are testing sight of the " + detector.getColor() + " mineral.");
        telemetry.addData("Sees Mineral?", detector.isFound());
        telemetry.addData("X Pos" , detector.getScreenPosition().x); // Mineral X position
        telemetry.addData("Y Pos" , detector.getScreenPosition().y); // Mineral Y position
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