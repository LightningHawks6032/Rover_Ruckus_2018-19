package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class Robot1_LinearActuatorTest extends OpMode {
    Robot1_Hardware hardware;

    public void init() {
        //Initialize hardware
        hardware = new Robot1_Hardware(hardwareMap);
        hardware.initHardware();
    }

    public void loop() {
        testActuator();
    }

    private void testActuator(){
        if(gamepad1.dpad_up){
            hardware.linearActuator.setPosition(1);
        }else if(gamepad1.dpad_down){
            hardware.linearActuator.setPosition(0);
        }
    }
}
