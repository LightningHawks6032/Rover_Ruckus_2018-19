package org.firstinspires.ftc.teamcode.Vision.DetectorTests;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Robot1_Hardware;
import org.firstinspires.ftc.teamcode.Robot1.Robot1_TeleOp;
import org.firstinspires.ftc.teamcode.Vision.Detectors.NavTargetDetector;
import java.lang.Math;


@TeleOp(name="Nav Target Detection Test", group="Iterative OpMode")
public class NavTargetDetectorTest extends OpMode
{
    // Detector object
    private NavTargetDetector detector;


    private final static int CAMERA_FORWARD_POSITION = 0, // eg: Camera is 0 mm in front of robot center
            CAMERA_VERTICAL_POSITION = 0, // eg: Camera is 0 mm above ground
            CAMERA_LEFT_POSITION = 0; // eg: Camera is 0 mm left of the robot's center line

    @Override
    public void init() {
        // Set up detector
        detector = new NavTargetDetector(hardwareMap, CAMERA_FORWARD_POSITION, CAMERA_VERTICAL_POSITION, CAMERA_LEFT_POSITION, true); // Create detector
        detector.setupTracker();
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
        detector.lookForTargets();

        telemetry.addData("Target Visible: ", detector.isTargetVisible());
        if (detector.isTargetVisible()) {
            telemetry.addData("The robot sees", detector.visibleTarget());
            telemetry.addData("Robot Pos", detector.getRobotPosition().toString());
            telemetry.addData("Cam roll", Math.round(detector.getCamRoll()));
            telemetry.addData("Cam pitch", Math.round(detector.getCamPitch()));
            telemetry.addData("Cam yaw", Math.round(detector.getCamYaw()));
            telemetry.addData("Robot rotation", Math.round(detector.getRobotRotation()));
        } else
            telemetry.addLine("The robot sees: No Target");
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }

}