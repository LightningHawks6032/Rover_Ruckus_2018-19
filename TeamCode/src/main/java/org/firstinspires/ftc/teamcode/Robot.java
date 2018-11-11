/**
 * This class is an unfinished Robot class which will store the robot's methods all in one place.
 * We may or may not use this, just an idea.
 * It is abstract, so we would have to make a subclass that supplies the code for the abstract methods.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.teamcode.Hardware.HardwareInterface;

public abstract class Robot {
    private String name;
    private HardwareInterface hardware;


    public Robot(String name, HardwareInterface hardware) {
        this.name = name;
        this.hardware = hardware;
    }

    public abstract void manageDrivetrain();
    public abstract void manageManips();



}
