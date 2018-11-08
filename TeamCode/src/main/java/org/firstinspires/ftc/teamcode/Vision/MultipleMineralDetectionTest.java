package org.firstinspires.ftc.teamcode.Vision;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;


@TeleOp(name="Multiple Mineral Detection Test", group="DogeCV")
public class MultipleMineralDetectionTest extends OpMode
{
    // Array of Detector object
    private ArrayList<MineralDetector> detectors;


    @Override
    public void init() {
        // Set up detectors
        detectors = new ArrayList<MineralDetector>();
        detectors.add(new MineralDetector("gold"));
        detectors.add(new MineralDetector("silver"));

        for (MineralDetector d : detectors)
            d.setupDetector(hardwareMap);
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
        // Output for all minerals
        telemetry.addLine("Testing:");
        for (MineralDetector d : detectors) {
            telemetry.addLine(d.getColor() + " mineral.");
            telemetry.addData("Sees " + d.getColor() + " Mineral?", d.isFound());
            telemetry.addData(d.getColor() + " mineral X Pos", d.getScreenPosition().x);
        }
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        // Disable the detector
        for (MineralDetector d : detectors)
            d.disable();
    }

}