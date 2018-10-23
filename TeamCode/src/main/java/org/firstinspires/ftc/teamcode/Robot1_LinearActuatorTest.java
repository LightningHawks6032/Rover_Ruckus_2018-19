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
        
    }
}
