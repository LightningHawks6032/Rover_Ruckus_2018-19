/**
 * Class for testing position tracking.
 */

package org.firstinspires.ftc.teamcode.AutoTesting;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FieldMapping.FieldMap;
import org.firstinspires.ftc.teamcode.Hardware.Robot1_Hardware;

public class Robot1_PositionTracking extends LinearOpMode {

    private Robot1_Hardware hardware;
    private FieldMap fieldMap = new FieldMap();

    public void runOpMode() {
        hardware = new Robot1_Hardware(hardwareMap, gamepad1, true);
        hardware.initHardware();
        hardware.navTargetDetector.setupTracker();

        waitForStart();

        telemetry.addLine("Place the robot in front of one of the navigation targets");
        telemetry.addLine("Robot updating position");
        telemetry.update();
        updatePosition();

        telemetry.addLine("Driving to Q3 Right Mineral");
        telemetry.update();
        hardware.drivetrain.goTo(fieldMap.get(3, "Left Mineral"), 0.6);

        telemetry.addLine("Driving to Q3 Depot");
        telemetry.update();
        hardware.drivetrain.goTo(fieldMap.get(3, "Red Depot"), 0.8);
    }

    private void updatePosition() {
        sleep(2000);
        hardware.navTargetDetector.lookForTargets();
        hardware.drivetrain.setRobotPos(hardware.navTargetDetector.getRobotPosition().getX(), hardware.navTargetDetector.getRobotPosition().getY());
        hardware.drivetrain.setRobotAngle((int) hardware.navTargetDetector.getRobotRotation());
    }
}
