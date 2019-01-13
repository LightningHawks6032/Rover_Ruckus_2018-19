/**
 * This class is a subclass of DogeCVDetector (a generic detector class), which I've written to take in the robot's center
 * x position based on the camera footage (essentially the x position of the mineral that causes x-alignment), as well as a
 * margin of error. Then isFound() returns true if a gold mineral is in the camera's frame. The method getXPosition()
 * returns what the x-position is in pixels on the screen is and getAligned() returns true if the robot is aligned with the
 * mineral based on the parameters specified.
 */

package org.firstinspires.ftc.teamcode.Vision.Detectors;

import android.util.Log;

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

//Based on com.disnodeteam.dogecv.detectors.roverrukus;

public class GoldAlignDetector extends DogeCVDetector {

    // Defining Mats to be used.
    private Mat displayMat = new Mat(); // Display debug info to the screen (this is what is returned)
    private Mat workingMat = new Mat(); // Used for pre-processing and working with (blurring as an example)
    private Mat maskYellow = new Mat(); // Yellow Mask returned by color filter
    private Mat hierarchy  = new Mat(); // hierarchy used by countours

    // Results of the detector
    private boolean found = false; // Is the gold mineral found
    private boolean aligned = false; // Is the gold mineral aligned
    private double goldXPos = 0; // X Position (in pixels) of the gold element
    private double goldYPos = 0; // Y Position (in pixels) of the gold element

    // Detector settings
    private int robotCenterX; // The x position in pixels of the center of the robot (find by placing gold mineral in front of robot at its center)
    private int yMax; // The max y position of a mineral (for avoiding minerals in crater)
    private boolean debugAlignment = true; // Show debug lines to show alignment settings
    private double alignSize; // How wide is the margin of error for alignment
    private boolean landscapeMode; // Is the phone using landscape mode

    private DogeCV.AreaScoringMethod areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Setting to decide to use MaxAreaScorer or PerfectAreaScorer


    //Create the default filters and scorers
    private DogeCVColorFilter yellowFilter = new LeviColorFilter(LeviColorFilter.ColorPreset.YELLOW); // Default Yellow filter
    private RatioScorer ratioScorer = new RatioScorer(1.0, 3); // Used to find perfect squares
    private MaxAreaScorer maxAreaScorer = new MaxAreaScorer( 0.01); // Used to find largest objects
    private PerfectAreaScorer perfectAreaScorer = new PerfectAreaScorer(5000,0.05); // Used to find objects near a tuned area value

    /**
     * Constructor builds a GoldAlignDetector by using the superclass
     * @param robotCenterX : the x position of the center of the robot
     * @param marginOfError : wideness of margin of error for alignment
     * @param landscape : boolean for whether we are using landscape mode
     */
    public GoldAlignDetector(int robotCenterX, int ym, double marginOfError, boolean landscape) {
        super();
        detectorName = "Gold Align Detector"; // Set the detector name

        this.robotCenterX = robotCenterX;
        yMax = ym;
        alignSize = marginOfError;
        landscapeMode = landscape;
    }


    @Override
    public Mat process(Mat input) {

        // Copy the input mat to our working mats, then release it for memory
        input.copyTo(displayMat);
        input.copyTo(workingMat);
        input.release();


        //Preprocess the working Mat (blur it then apply a yellow filter)
        Imgproc.GaussianBlur(workingMat,workingMat,new Size(5,5),0);
        yellowFilter.process(workingMat.clone(),maskYellow);

        //Find contours of the yellow mask and draw them to the display mat for viewing

        List<MatOfPoint> contoursYellow = new ArrayList<>();
        Imgproc.findContours(maskYellow, contoursYellow, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(displayMat,contoursYellow,-1,new Scalar(230,70,70),2);

        // Current result
        Rect bestRect = null;
        double bestDifference = Double.MAX_VALUE; // MAX_VALUE since less diffrence = better

        // Loop through the contours and score them, searching for the best result
        for(MatOfPoint cont : contoursYellow){
            double score = calculateScore(cont); // Get the diffrence score using the scoring API

            // Get bounding rect of contour
            Rect rect = Imgproc.boundingRect(cont);
            Imgproc.rectangle(displayMat, rect.tl(), rect.br(), new Scalar(0,0,255),2); // Draw rect

            // If the result is better then the previously tracked one, set this rect as the new best
            if(score < bestDifference && rect.x < yMax) { // since we're in landscape mode, x compared to y max
                bestDifference = score;
                bestRect = rect;
            }
        }

        // Vars to calculate the alignment logic.
        double alignX = robotCenterX;
        double alignXMin = alignX - (alignSize / 2); // Min X Pos in pixels
        double alignXMax = alignX +(alignSize / 2); // Max X pos in pixels
        double xPos; // Current Gold X Pos
        double yPos; // Current Gold Y Pos

        if(bestRect != null && bestRect.x < yMax){ // since we're in landscape mode, x compared to y max
            // Show chosen result
            Imgproc.rectangle(displayMat, bestRect.tl(), bestRect.br(), new Scalar(255,0,0),4);
            Imgproc.putText(displayMat, "Chosen", bestRect.tl(),0,1,new Scalar(255,255,255));

            // Set align X pos based on whether or not landscape mode used
            if (landscapeMode) {
                xPos = bestRect.y + (bestRect.height / 2);
                yPos = bestRect.x + (bestRect.width / 2);
            } else {
                xPos = bestRect.x + (bestRect.width / 2);
                yPos = bestRect.y + (bestRect.height / 2);
            }
            goldXPos = xPos;
            goldYPos = yPos;

            // Draw center point
            Imgproc.circle(displayMat, new Point( xPos, bestRect.y + (bestRect.height / 2)), 5, new Scalar(0,255,0),2);

            // Check if the mineral is aligned
            if (xPos < alignXMax && xPos > alignXMin){
                aligned = true;
            } else{
                aligned = false;
            }

            found = true;
        }else{
            found = false;
            aligned = false;
        }
        if(debugAlignment){

            //Draw debug alignment info
            if(isFound()){
                Imgproc.line(displayMat,new Point(goldXPos, getAdjustedSize().height), new Point(goldXPos, getAdjustedSize().height - 30),new Scalar(255,255,0), 2);
            }

            Imgproc.line(displayMat,new Point(alignXMin, getAdjustedSize().height), new Point(alignXMin, getAdjustedSize().height - 40),new Scalar(0,255,0), 2);
            Imgproc.line(displayMat,new Point(alignXMax, getAdjustedSize().height), new Point(alignXMax,getAdjustedSize().height - 40),new Scalar(0,255,0), 2);
        }


        return displayMat;

    }

    @Override
    public void useDefaults() {
        addScorer(ratioScorer);

        // Add different scorers depending on the selected mode
        if(areaScoringMethod == DogeCV.AreaScoringMethod.MAX_AREA){
            addScorer(maxAreaScorer);
        }

        if (areaScoringMethod == DogeCV.AreaScoringMethod.PERFECT_AREA){
            addScorer(perfectAreaScorer);
        }

    }

    /**
     * Returns if the gold element is aligned
     * @return if the gold element is alined
     */
    public boolean getAligned(){
        return aligned && found;
    }

    /**
     * Returns gold element last x-position
     * @return last x-position in screen pixels of gold element
     */
    public double getXPosition(){
        return goldXPos;
    }

    /**
     * Returns gold element last y-position
     * @return last y-position in screen pixels of gold element
     */
    public double getYPosition() {
        return goldYPos;
    }

    /**
     * Returns if a gold mineral is being tracked/detected
     * @return if a gold mineral is being tracked/detected
     */
    public boolean isFound() {
        return found;
    }

    /**
     * If the camera is far back enough so that all minerals are in view, this method can determine the location of the gold mineral
     * @return location of gold mineral (1 = left, 2 = center, 3 = right); default is center
     */
    public int mineralLocation() {
        if (getAligned())
            return 2;
        else {
            if (robotCenterX < getXPosition())
                return 3;
            else if (robotCenterX > getXPosition())
                return 1;
        }

        return 2;
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
    public double getRobotCenterX(){
        return robotCenterX;
    }
}
