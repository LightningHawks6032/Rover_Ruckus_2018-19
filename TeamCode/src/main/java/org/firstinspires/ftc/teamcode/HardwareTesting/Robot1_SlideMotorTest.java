package org.firstinspires.ftc.teamcode.HardwareTesting;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Robot1_Hardware;

@TeleOp(name="Robot1 Slide Test", group="Iterative Opmode")
public class Robot1_SlideMotorTest extends OpMode {
    Robot1_Hardware hardware;


    // currently 1.0 is optimal
    double powerCoefficient = 0.3; // we're testing this value

    double redMotorPower = 1.0; // RED = FAST
    double blueMotorPower = redMotorPower * powerCoefficient;

    public void init() {
        //Initialize hardware
        hardware = new Robot1_Hardware(hardwareMap);
        hardware.initHardware();
    }

    public void loop() {
        redMotorPower = -gamepad1.left_stick_y;
        blueMotorPower = redMotorPower * powerCoefficient;

        if (gamepad1.dpad_down)
            powerCoefficient -= 0.005;
        else if (gamepad1.dpad_up)
            powerCoefficient += 0.005;

        if (powerCoefficient <= 0) {
            powerCoefficient = 0;
        }
        if (powerCoefficient >= 1) {
            powerCoefficient = 1;
        }


        // Update motor power
        hardware.fastSlideMotor.setPower(redMotorPower);
        hardware.slowSlideMotor.setPower(blueMotorPower);


        // Print to telemetry
        telemetry.addLine("Up/down on dpad for 0.05 increase/decrease in power coefficient");
        telemetry.addData("Fast (Red) Motor", hardware.slowSlideMotor.getPower());
        telemetry.addData("Slow (Blue) Motor", hardware.slowSlideMotor.getPower());
        telemetry.addData("Power Coefficient (Ratio between slow and fast power)", powerCoefficient);
        telemetry.update();
    }
}
