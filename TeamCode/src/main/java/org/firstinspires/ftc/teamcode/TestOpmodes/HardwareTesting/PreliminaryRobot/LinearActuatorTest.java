package org.firstinspires.ftc.teamcode.TestOpmodes.HardwareTesting.PreliminaryRobot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.PrelimBot_Hardware;

// @TeleOp(name="Robot1 Linear Actuator Test", group="Iterative Opmode")
public class LinearActuatorTest extends OpMode {
    PrelimBot_Hardware hardware;

    double actuPos;

    public void init() {
        //Initialize hardware
        hardware = new PrelimBot_Hardware(hardwareMap, gamepad1, false);
        hardware.initHardware();
        actuPos = 0.5;
    }

    public void loop() {
        testActuator();
        telemetry.addLine("This program tests the linear actuator as a servo");
        telemetry.addData("Linear actuator position: ",hardware.linearActuator.getPosition());
        telemetry.addData("Actupos variable", actuPos);
        telemetry.update();
    }

    private void testActuator(){
        /*if(gamepad2.dpad_up) {
            actuPos += 0.01;
        }else if(gamepad2.dpad_down) {
            actuPos -= 0.01;
        }else{
            // I'm hoping this will be able to make the linear actuator sit still - Mich
            actuPos = hardware.linearActuator.getPosition();
            if(actuPos < 0) {
                actuPos = 0;
            }
            if(actuPos > 1){
                actuPos = 1;
            }
        }*/

        actuPos -= gamepad2.right_stick_y * 0.01;

        if(actuPos > 1){
            actuPos = 1;
        }
        if(actuPos < 0){
            actuPos = 0;
        }

        hardware.linearActuator.setPosition(actuPos);
    }
}
