package org.firstinspires.ftc.teamcode.Vision.DetectorTests;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Robot1_Hardware;
import org.firstinspires.ftc.teamcode.Robot1_TeleOp;
import org.firstinspires.ftc.teamcode.Vision.Detectors.NavTargetDetector;


@TeleOp(name="Nav Target Detection Test", group="Iterative OpMode")
public class NavTargetDetectorTest extends OpMode
{
    // Detector object
    private NavTargetDetector detector;

    // Hardware object
    Robot1_Hardware hardware = new Robot1_Hardware(hardwareMap);

    @Override
    public void init() {
        // Set up detector
        detector = new NavTargetDetector(hardwareMap, hardware.CAMERA_FORWARD_POSITION, hardware.CAMERA_VERTICAL_POSITION, hardware.CAMERA_LEFT_POSITION); // Create detector
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
        detector.lookForDetectors();

        telemetry.addData("Target Visible: ", detector.isTargetVisible());
        if (detector.isTargetVisible())
            telemetry.addData("The robot sees", detector.visibleTarget());
        else
            telemetry.addLine("The robot sees: No Target");
        telemetry.addData("Robot Pos" , detector.getRobotPosition());
        telemetry.addData("Robot rotation", detector.getRobotRotation());
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }

}