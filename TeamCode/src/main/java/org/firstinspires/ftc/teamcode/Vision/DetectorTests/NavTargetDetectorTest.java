package org.firstinspires.ftc.teamcode.Vision.DetectorTests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.QualBot_Hardware;
import org.firstinspires.ftc.teamcode.Hardware.StatesBot_Hardware;
import org.firstinspires.ftc.teamcode.Vision.Detectors.NavTargetDetector;
import java.lang.Math;


@TeleOp(name="Nav Target Detection Test", group="Iterative OpMode")
public class NavTargetDetectorTest extends OpMode
{
    // Detector object
    private NavTargetDetector detector;
    private StatesBot_Hardware hardware;

    @Override
    public void init() {
        // Set up detector
        hardware = new StatesBot_Hardware(hardwareMap, gamepad1, gamepad2, false);
        hardware.initHardware();
        detector = hardware.navTargetDetector; // Create detector
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
        hardware.drivetrain.manageTeleOp();
        detector.lookForTargets();

        telemetry.addData("Target Visible: ", detector.isTargetVisible());
        if (detector.isTargetVisible()) {
            telemetry.addData("The robot sees", detector.visibleTarget());
            telemetry.addData("Cam Pos", detector.getCamPosition().toString());
            telemetry.addData("Robot Pos", detector.getRobotPosition().toString());
            telemetry.addData("Quadrant", detector.getRobotPosition().quadrant());
            telemetry.addData("Robot Rotation", Math.round(detector.getRobotRotation()));
            telemetry.addData("Detector FirstAngle", detector.robotRotation.firstAngle);
            telemetry.addData("Detector SecondAngle", detector.robotRotation.secondAngle);
            telemetry.addData("Detector ThirdAngle", detector.robotRotation.thirdAngle);
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