/**
 * This class is a subclass of DogeCVDetector (a generic detector class), which I've written to take in a color mineral
 * (either silver or gold) as a string in its constructor. Then isFound() and getScreenPosition() return whether or not
 * the mineral is in the camera's frame and if so, what the x-position is in pixels on the screen is.
 */

package org.firstinspires.ftc.teamcode.Vision.Detectors;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.DogeCVDetector;
import com.disnodeteam.dogecv.filters.DogeCVColorFilter;
import com.disnodeteam.dogecv.filters.LeviColorFilter;
import com.disnodeteam.dogecv.scoring.MaxAreaScorer;
import com.disnodeteam.dogecv.scoring.PerfectAreaScorer;
import com.disnodeteam.dogecv.scoring.RatioScorer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class MineralDetector extends DogeCVDetector {

    private String color; // color mineral we're trying to detect
    private boolean lanscapeMode; // Is the phone using landscape mode

    // Defining Mats to be used
    private Mat displayMat = new Mat(); // Display debug info to the screen (this is what is returned)
    private Mat workingMat = new Mat(); // Used for preprocessing and working with (blurring as an example)
    private Mat mask = new Mat(); // Mask returned by color filter
    private Mat hierarchy = new Mat(); // hierarchy used by contours

    // Results of the detector
    private boolean found = false; // Is the mineral found?
    private Point screenPosition = new Point(); // Screen position of the mineral (in px)
    private Rect foundRect = new Rect(); // Found rect

    // Setting to decide to use MaxAreaScorer or PerfectAreaScorer
    public DogeCV.AreaScoringMethod areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA;

    //Create the default filters and scorers
    public DogeCVColorFilter colorFilter; //Color filter for color of filter

    public RatioScorer       ratioScorer       = new RatioScorer(1.0, 3);          // Used to find perfect squares
    public MaxAreaScorer     maxAreaScorer     = new MaxAreaScorer( 0.01);                    // Used to find largest objects
    public PerfectAreaScorer perfectAreaScorer = new PerfectAreaScorer(5000,0.05); // Used to find objects near a tuned area value

    // Constructor takes in color as a string
    public MineralDetector(String color, boolean landscape) {
        super(); // Call constructor of general detector class (which this is a subclass of)
        detectorName = "Mineral Detector"; // Set the detector name
        this.color = color;

        if (color.equals("gold")) // GOLD MINERAL (Color: Yellow)
            colorFilter = new LeviColorFilter(LeviColorFilter.ColorPreset.YELLOW);
        else if (color.equals("silver")) // SILVER MINERAL (Color: White)
            colorFilter = new LeviColorFilter(LeviColorFilter.ColorPreset.WHITE);
        else // DEFAULT: RED
            colorFilter = new LeviColorFilter(LeviColorFilter.ColorPreset.RED);

        lanscapeMode = landscape;

    }


    /* The following method processes the screen input, scans for mineral, and updates found bool accordingly */
    @Override
    public Mat process(Mat input) {
        // Copy the input mat to our working mats, then release it for memory
        input.copyTo(displayMat);
        input.copyTo(workingMat);
        input.release();

        // Preprocess the working Mat (blur it then apply a color filter)
        Imgproc.GaussianBlur(workingMat,workingMat,new Size(5,5),0);
        colorFilter.process(workingMat.clone(),mask      );

        // Find contours of the yellow mask and draw them to the display mat for viewing
        List<MatOfPoint> contoursYellow = new ArrayList<>();
        Imgproc.findContours(mask      , contoursYellow, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(displayMat,contoursYellow,-1,new Scalar(230,70,70),2);

        // Current result
        Rect bestRect = null;
        double bestDiffrence = Double.MAX_VALUE; // MAX_VALUE since less diffrence = better

        // Loop through the contours and score them, searching for the best result
        for(MatOfPoint cont : contoursYellow){
            double score = calculateScore(cont); // Get the diffrence score using the scoring API

            // Get bounding rect of contour
            Rect rect = Imgproc.boundingRect(cont);
            Imgproc.rectangle(displayMat, rect.tl(), rect.br(), new Scalar(0,0,255),2); // Draw rect

            // If the result is better then the previously tracked one, set this rect as the new best
            if(score < bestDiffrence){
                bestDiffrence = score;
                bestRect = rect;
            }
        }

        if (bestRect != null) {
            // Show chosen result
            Imgproc.rectangle(displayMat, bestRect.tl(), bestRect.br(), new Scalar(255,0,0),4);
            Imgproc.putText(displayMat, "Chosen", bestRect.tl(),0,1,new Scalar(255,255,255));

            if (lanscapeMode)
                screenPosition = new Point(bestRect.y, bestRect.x);
            else
                screenPosition = new Point(bestRect.x, bestRect.y);
            foundRect = bestRect;
            found = true;
        } else {
            found = false;
        }


        //Print result
        Imgproc.putText(displayMat,"Result: " + screenPosition.x +"/"+screenPosition.y,new Point(10,getAdjustedSize().height - 30),0,1, new Scalar(255,255,0),1);

        return displayMat;

    }

    @Override
    public void useDefaults() {
        addScorer(ratioScorer);

        // Add different scoreres depending on the selected mode
        if(areaScoringMethod == DogeCV.AreaScoringMethod.MAX_AREA){
            addScorer(maxAreaScorer);
        }

        if (areaScoringMethod == DogeCV.AreaScoringMethod.PERFECT_AREA){
            addScorer(perfectAreaScorer);
        }

    }

    // Return's the mineral's last position in pixels
    public Point getScreenPosition() {
        return screenPosition;
    }

    // Returns the rectangle to surround mineral on screen
    public Rect getFoundRect() {
        return foundRect;
    }

    // Returns if the mineral has been tracked/detected
    public boolean isFound() {
        return found;
    }

    // Return color we're currently trying to find
    public String getColor() {
        return color;
    }

    public void setupDetector(HardwareMap hwMap, int cameraIndex) {
        // Camera Index: 0 for back camera, 1 for front camera
        init(hwMap.appContext, CameraViewDisplay.getInstance(), cameraIndex, false); // Initialize it with the app context and camera
        useDefaults(); // Set detector to use default settings

        // Optional tuning
        downscale = 0.4; // How much to downscale the input frames

        areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        maxAreaScorer.weight = 0.005; //

        ratioScorer.weight = 5; //
        ratioScorer.perfectRatio = 1.0; // Ratio adjustment

        enable(); // Start the detector!
    }
}
