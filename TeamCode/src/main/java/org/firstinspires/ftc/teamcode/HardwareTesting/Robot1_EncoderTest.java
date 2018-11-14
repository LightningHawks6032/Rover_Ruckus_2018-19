package org.firstinspires.ftc.teamcode.HardwareTesting;

import org.firstinspires.ftc.teamcode.Hardware.Encoder;
import org.firstinspires.ftc.teamcode.Hardware.Robot1_Hardware;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Robot 1 Encoder Test", group="Iterative Opmode")
public class Robot1_EncoderTest extends OpMode {
    Robot1_Hardware hardware;

    Encoder leftDriveEncoder;
    Encoder rightDriveEncoder;

    public void init() {
        //Initialize Hardware
        hardware = new Robot1_Hardware(hardwareMap);
        hardware.initHardware();

        leftDriveEncoder = new Encoder(hardware.leftDrive, "Neverest", 4);
        rightDriveEncoder = new Encoder(hardware.rightDrive, "Neverest", 4);
    }

    public void loop() {
        basicDrive();
        double avg = (leftDriveEncoder.linDistance() + rightDriveEncoder.linDistance())/2;
        telemetry.addData("Left Encoder count: ", leftDriveEncoder.getEncoderCount());
        telemetry.addData("Left Motor rotations: ", leftDriveEncoder.motorRotations());
        telemetry.addData("Right Encoder count: ", rightDriveEncoder.getEncoderCount());
        telemetry.addData("Right Motor rotations: ", rightDriveEncoder.motorRotations());
        telemetry.addData("Distance: ", avg);
        telemetry.update();

    }

    // Shortcut method for setting the power of the left drive, right drive, and middle drive
    private void setPowers(double lp, double rp, double mp) {
        hardware.leftDrive.setPower(lp);
        hardware.rightDrive.setPower(rp);
        hardware.middleDrive.setPower(mp);
    }

    private void basicDrive() {
        //drive1 controls (slide drive)
        if (gamepad1.left_trigger > 0) {
            setPowers(0, 0, -1); // strafe left
        } else if (gamepad1.right_trigger > 0) {
            setPowers(0, 0, 1); // strafe right
        } else {
            // motor power = joysticks
            setPowers(-gamepad1.left_stick_y, -gamepad1.right_stick_y, 0);
        }
    }
}
