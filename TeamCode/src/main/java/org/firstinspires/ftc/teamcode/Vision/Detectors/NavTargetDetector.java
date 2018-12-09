/**
 * This class provides the layout for a navigation target detector object, and can be instantiated in an autonomous or tele-op
 * mode to return robot's current position and rotation. Much of the code I've written is based on VuforiaTesting.java, which was a
 * simple opmode for me to print out vuforia return values to telemetry.
 */

package org.firstinspires.ftc.teamcode.Vision.Detectors;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.FieldMapping.Vector;

import java.util.ArrayList;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

public class NavTargetDetector {
    // Constant license key
    private static final String VUFORIA_KEY = "AdwaKe7/////AAAAmVQWX/gUQE/gnK+olEmSWA5FCaxNrdY/EyKFLO2afR1IQD4gbnThc6LcCHIJ64hyC2i3n5VRiIRAMGxtKqjI7meHCphQAPrXpH9GomENr/fSXjVUhQao+Zw0/MLQEuTaqNYnp5EI/4oo6LTm/YPgYKOSPaP+tijaydiwNQn4A8zXPfDhkD/q6RTYMzS3UtpOR7WBZJPUBxW9XKim5ekHbYd1Hk2cFTTFAsL0XwycIWhuvHYpVlnZMqWwEnkTqp0o+5TE1FLkAfJ4OOUEfB8sP9kMEcged2/tczAh3GOcjOudp1S9F5xjPFZQX00OLV+QUCPzmT5kkqFBwiS30YR6L8urW2mJG4quq6NnrNYwzn47";

    // Since ImageTarget trackables use mm to specify their dimensions, we must use mm for all the physical dimension.
    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch; // width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch; // height of the center of the target image above the floor

    // The camera on the RC that we are using (FRONT or BACK)
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = FRONT;

    // Setup tracking data
    private VuforiaLocalizer vuforia;
    private ArrayList<VuforiaTrackable> navigationTargets;

    // Instance data: hardwareMap and three vertical displacements that indicate phone location on robot
    private HardwareMap hwMap;
    private int camForwardDisplacement; // eg: Camera is 0 mm in front of robot center
    private int camVerticalDisplacement; // eg: Camera is 0 mm above ground
    private int camLeftDisplacement; // eg: Camera is ON the robot's center line

    // For returning to telemetry
    private boolean targetVisible;
    private String whichTargetVisible;
    private static final String[] targetNames = {"Blue-Rover", "Red-Footprint", "Front-Craters", "Back-Space"};
    private OpenGLMatrix lastLocation = null; // the last location of a nav target we've seen
    private VectorF robotPos;
    private Orientation robotRotation;

    public NavTargetDetector(HardwareMap hwMap, int camForwardDisplacement, int camVerticalDisplacement, int camLeftDisplacement) {
        this.hwMap = hwMap;
        navigationTargets = new ArrayList<VuforiaTrackable>();
        this.camForwardDisplacement = camForwardDisplacement;
        this.camVerticalDisplacement = camVerticalDisplacement;
        this.camLeftDisplacement = camLeftDisplacement;

        targetVisible = false; // by default, we assume we don't see a target
        whichTargetVisible = null; // by default, we assume we don't see a target
        robotPos = null;
        robotRotation = null;
    }

    public void setupTracker() {
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CAMERA_CHOICE;

        // Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable navigation targets
        VuforiaTrackables targetsRoverRuckus = vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName(targetNames[0]);
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName(targetNames[1]);
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName(targetNames[2]);
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName(targetNames[3]);

        // Because of landscape mode:
        // New roll = previous roll
        // New pitch = previous yaw
        // New yaw = previous pitch

        /** Blue Rover Target, Middle of Blue Perimeter Wall **/
        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);

        /** Red Footprint Target, Middle of Red Perimeter Wall **/
        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        redFootprint.setLocation(redFootprintLocationOnField);

        /** Front Craters Target, Middle of Front Perimeter Wall **/
        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        frontCraters.setLocation(frontCratersLocationOnField);

        /** Back Space Target, Middle of Back Perimeter Wall **/
        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        backSpace.setLocation(backSpaceLocationOnField);

        // Store the navigation targets in the navigationTargets ArrayList
        navigationTargets.addAll(targetsRoverRuckus);

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(camForwardDisplacement, camLeftDisplacement, camVerticalDisplacement)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        CAMERA_CHOICE == FRONT ? 90 : -90 /*0*/, 0, 0));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : navigationTargets) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }

        targetsRoverRuckus.activate();
    }

    public void lookForTargets() {
        for (VuforiaTrackable trackable : navigationTargets) {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                targetVisible = true;
                whichTargetVisible = trackable.getName();

                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break; // Exit the for loop if we've found one of the nav targets
            }
        }

        // If we've seen a target and know where the robot is
        if (targetVisible && lastLocation != null) {
            // express position (translation) of robot in inches.
            robotPos = lastLocation.getTranslation();

            // Express the rotation of the robot in degrees. Extrinsic = Roll, XYZ = Pitch, DEGREES = Yaw
            // More info about Roll, Pitch, Yaw: https://www.novatel.com/assets/Web-Phase-2-2012/Solution-Pages/AttitudePlane.png
            robotRotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
        }
    }

    // The following methods check for the visibility of a specific target by inputting its index in navigationTargets or target name
    public boolean specificTargetVisible(int targetIndex) {
        return ((VuforiaTrackableDefaultListener) navigationTargets.get(targetIndex).getListener()).isVisible();
    }
    /*public boolean specificTargetVisible(String targetName) {
        return ((VuforiaTrackableDefaultListener) navigationTargets.get(navigationTargets.indexOf(targetName)).getListener()).isVisible();
    }*/

    // Return true if the robot sees any target at all
    public boolean isTargetVisible() {
        return targetVisible;
    }

    // Return which target the robot sees
    public String visibleTarget() {
        return whichTargetVisible;
    }

    // Returns vector of robot's position in inches
    public Vector getRobotPosition() {
        double x = robotPos.get(0) / mmPerInch, y = robotPos.get(1) / mmPerInch;

        // Craters
        if (specificTargetVisible(2)) {
            return new Vector(y, -x);
        }

        // Footprint
        else if (specificTargetVisible(1)) {
            return new Vector(-x, -y);
        }

        // Space
        else if (specificTargetVisible(3)) {
            return new Vector(-y, x);
        }

        // Rover
        return new Vector(x, y);
    }

    // Returns X coordinate
    public double getX(){
        return robotPos.get(0);
    }

    // Returns Y coordinate
    public double getY(){
        return robotPos.get(1);
    }

    public double getCamRoll() {
        return robotRotation.firstAngle;
    }
    public double getCamPitch() {
        return robotRotation.secondAngle;
    }
    public double getCamYaw() {
        return robotRotation.thirdAngle;
    }

    // Returns robot's rotation in degrees --> only rotational component we care about
    public double getRobotRotation() {
        double yaw = robotRotation.thirdAngle;

        // Craters
        if (specificTargetVisible(2)) {
            if (yaw - 90 < 0)
                return 270 + yaw;
            else
                return yaw - 90;
        }

        // Footprint
        else if (specificTargetVisible(1)) {
            return yaw + 180;
        }

        // Space
        else if (specificTargetVisible(3)) {
            return yaw + 90;
        }

        // Rover
        return robotRotation.thirdAngle;
    }
}
