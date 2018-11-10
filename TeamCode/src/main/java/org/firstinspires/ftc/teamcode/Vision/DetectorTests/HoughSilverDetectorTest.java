package org.firstinspires.ftc.teamcode.Vision.DetectorTests;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.roverrukus.HoughSilverDetector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Size;


@TeleOp(name="Hough Silver Detector Test", group="DogeCV")
public class HoughSilverDetectorTest extends OpMode
{
    //Detector object
    private HoughSilverDetector detector;


    @Override
    public void init() {
        detector = new HoughSilverDetector(1.8); //Create detector
        detector.downscale = 1; //Increase detector sensitivity with smaller size. Make sure to preserve aspect ratio.
        detector.useFixedDownscale = false; //Don't fix the downscale
        detector.minDistance = 60; //Minimum distance between silver mineral centers in pixels
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance()); //Initialize detector with app context and camera
        detector.useDefaults(); //Use default settings

        // Optional Tuning
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.enable(); //Start the detector
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
        telemetry.addData("Found Silver Mineral", detector.isFound());
        if (detector.isFound())
            telemetry.addData("Found Circle", detector.getFoundCircle().toString());
        else
            telemetry.addLine("No Found Circle");
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        detector.disable();
    }

}