/**
 * Basic auto class to test the turn() function(s) in the drivetrain class which use gyro
 */

package org.firstinspires.ftc.teamcode.TestOpmodes.AutoTesting.PreliminaryRobot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.PrelimBot_Hardware;

// @Autonomous(name="Robot 1 Turning with Gyro", group=AutonomousData.TEST_GROUP)
public class TurningWithGyro extends LinearOpMode {
    private PrelimBot_Hardware hardware;

    public void runOpMode() {
        hardware = new PrelimBot_Hardware(hardwareMap, gamepad1, true);
        hardware.initHardware(true);

        waitForStart();

        telemetry.addLine("Turning right 90deg");
        telemetry.update();
        hardware.drivetrain.turn(90, true);

        telemetry.addLine("Turning left 90deg");
        telemetry.update();
        hardware.drivetrain.turn(90, false);
    }
}
