package org.firstinspires.ftc.teamcode.Vision.DetectorTests;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Robot1_Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Robot2_Hardware;
import org.firstinspires.ftc.teamcode.Robot1.Robot1_TeleOp;
import org.firstinspires.ftc.teamcode.Vision.Detectors.NavTargetDetector;
import java.lang.Math;


@TeleOp(name="Nav Target Detection Test", group="Iterative OpMode")
public class NavTargetDetectorTest extends OpMode
{
    // Detector object
    private NavTargetDetector detector;
    private Robot2_Hardware hardware;

    @Override
    public void init() {
        // Set up detector
        hardware = new Robot2_Hardware(hardwareMap, gamepad1, gamepad2, false);
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
        testPhoneServo();
        detector.lookForTargets();

        telemetry.addData("Phone Servo Position", hardware.phoneServo.getPosition());
        telemetry.addData("Target Visible: ", detector.isTargetVisible());
        if (detector.isTargetVisible()) {
            telemetry.addData("The robot sees", detector.visibleTarget());
            telemetry.addData("Cam Pos", detector.getCamPosition().toString());
            telemetry.addData("Robot Pos", detector.getRobotPosition().toString());
            telemetry.addData("Quadrant", detector.getRobotPosition().quadrant());
            telemetry.addData("Robot rotation", Math.round(detector.getRobotRotation()));
        } else
            telemetry.addLine("The robot sees: No Target");
        telemetry.update();
    }


    private void testPhoneServo() {
        if (gamepad1.dpad_up)
            hardware.phoneServo.setPosition(0);
        else if (gamepad1.dpad_right)
            hardware.phoneServo.setPosition(0.25);
        else if (gamepad1.dpad_down)
            hardware.phoneServo.setPosition(0.5);
        else if (gamepad1.dpad_left)
            hardware.phoneServo.setPosition(0.75);
        else if (gamepad1.x)
            hardware.phoneServo.setPosition(1);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }

}