package org.firstinspires.ftc.teamcode.TestOpmodes.HardwareTesting;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.OfficialBot_Hardware;
import org.firstinspires.ftc.teamcode.Hardware.OfficialBot_Intake;
import org.firstinspires.ftc.teamcode.Hardware.OfficialBot_Outtake;

@TeleOp(name="Dumper Servo Test", group="Test Opmode")
public class DumperServoTest extends OpMode{

    //for testing positions and difference between positions, i guess

    private OfficialBot_Hardware hardware;
    private OfficialBot_Outtake outtake;
    private double rightPos = 1;
    private double leftPos = 0.5;
    private double posDif = 0.4;

    public void init(){
        hardware = new OfficialBot_Hardware(hardwareMap, gamepad1, gamepad2, false);
        hardware.initHardware();
        // requires dumpers to not be initted in init
        outtake.rightDumper.setPosition(rightPos);
        outtake.leftDumper.setPosition(leftPos);
        outtake = hardware.outtake;
    }

    public void loop(){
        if(gamepad2.dpad_up) {
            outtake.rightDumper.setPosition(rightPos);
            outtake.leftDumper.setPosition(leftPos);
        }
        else if(gamepad2.dpad_down){
            outtake.rightDumper.setPosition(rightPos-posDif);
            outtake.leftDumper.setPosition(leftPos+posDif);
        }

    }
}
