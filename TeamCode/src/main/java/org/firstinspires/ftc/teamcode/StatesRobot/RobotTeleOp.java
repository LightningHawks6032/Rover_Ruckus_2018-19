package org.firstinspires.ftc.teamcode.StatesRobot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.QualBot_Hardware;
import org.firstinspires.ftc.teamcode.Hardware.StatesBot_Hardware;

@TeleOp(name="States Robot TeleOp", group="Iterative Opmode")
public class RobotTeleOp extends OpMode {
    private StatesBot_Hardware hardware;
    private boolean slideLimitingOn = true;

    public void init() {
        // init hardware
        hardware = new StatesBot_Hardware(hardwareMap, gamepad1, gamepad2, false);
        hardware.initHardware();
    }

    public void loop() {
        hardware.drivetrain.manageTeleOp();
        hardware.intake.manageTeleOp(slideLimitingOn);
        hardware.outtake.manageTeleOp(slideLimitingOn);
        manageDriverControlledDumper();
        manageSlideLimiting();
        debug();
    }

    private void debug() {
        telemetry.addData("Left Front Pow", hardware.drivetrain.leftFront.getPower());
        telemetry.addData("Right Front Pow", hardware.drivetrain.rightFront.getPower());
        telemetry.addData("Left Back Pow", hardware.drivetrain.leftBack.getPower());
        telemetry.addData("Right Back Pow", hardware.drivetrain.rightBack.getPower());
        telemetry.addData("Left Front Dist", hardware.drivetrain.leftFrontEncoder.linDistance());
        telemetry.addData("Right Front Dist", hardware.drivetrain.rightFrontEncoder.linDistance());
        telemetry.addData("Left Back Dist", hardware.drivetrain.leftBackEncoder.linDistance());
        telemetry.addData("Right Back Dist", hardware.drivetrain.rightBackEncoder.linDistance());
        telemetry.update();
    }

    // Booleans to manage dumping for tele-op with drivetrain controller
    private boolean dumperIn = true; // Should the dumper be flipping inward? (i.e. was the last command to flip inward?)
    private boolean togglePressed = false; // Is the toggle button currently pressed?
    private boolean toggleLastPressed = false; // Was the toggle button pressed last iteration of loop()?
    private void manageDriverControlledDumper() {
        // Use right bumper to toggle between dumper in and dumper out
        togglePressed = gamepad1.right_bumper;
        if (togglePressed && !toggleLastPressed) // Only change flipper if toggle button wasn't pressed last iteration of loop()
            dumperIn = !dumperIn;
        toggleLastPressed = togglePressed; // toggleLastPressed updated for the next iteration of loop()

        // Manage dump servo
        if (dumperIn) {
            hardware.outtake.leftDumper.setPosition(hardware.outtake.LEFT_DUMPER_IN);
            hardware.outtake.rightDumper.setPosition(hardware.outtake.RIGHT_DUMPER_IN);
        } else {
            hardware.outtake.leftDumper.setPosition(hardware.outtake.LEFT_DUMPER_OUT);
            hardware.outtake.rightDumper.setPosition(hardware.outtake.RIGHT_DUMPER_OUT);
        }
    }

    private void manageSlideLimiting() {
        if (gamepad1.dpad_down && gamepad1.x)
            slideLimitingOn = false;
    }
}
