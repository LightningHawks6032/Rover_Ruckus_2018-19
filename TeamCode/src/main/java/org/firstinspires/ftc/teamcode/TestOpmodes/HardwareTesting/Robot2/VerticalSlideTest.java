package org.firstinspires.ftc.teamcode.TestOpmodes.HardwareTesting.Robot2;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Robot2_Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Robot2_Outtake;

@TeleOp(name="Robot 2 Vertical Slide Test", group="Opmode")
public class VerticalSlideTest extends OpMode{
    private Robot2_Hardware hardware;
    private Robot2_Outtake outtake;

    private double leftPow, rightPow;

    public void init(){
        hardware = new Robot2_Hardware(hardwareMap, gamepad1, gamepad2, false);
        hardware.initHardware();
        outtake = hardware.outtake;
    }

    public void loop(){

    }
}
