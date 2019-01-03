package org.firstinspires.ftc.teamcode.Robot2;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.Robot2_Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Robot2_Outtake;
import org.firstinspires.ftc.teamcode.Hardware.Robot2_Intake;

@TeleOp(name="Robot2 TeleOp", group="Iterative Opmode" )
public class Robot2_TeleOp extends OpMode {
    private Robot2_Hardware hardware;

    public void init(){
        // init hardware
        hardware = new Robot2_Hardware(hardwareMap, gamepad1, gamepad2, false);
        hardware.initHardware();

        // init power variables

    }

    public void loop(){
        hardware.drivetrain.manageTeleOp();
        hardware.intake.manageTeleOp();
        hardware.outtake.manageTeleOp();
    }

    private void debug(){
        telemetry.addData("Left Motor Pow", hardware.drivetrain.getLeftPow());
        telemetry.addData("Left Encoder Val", hardware.drivetrain.getLeftEncoder().getEncoderCount());
        telemetry.addData("Right Motor Pow", hardware.drivetrain.getRightPow());
        telemetry.addData("Right Encoder Val", hardware.drivetrain.getRightEncoder().getEncoderCount());
        telemetry.update();
    }

    public void manageMarker(){

    }
}
