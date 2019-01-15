/**
 * An interface to be implemented by every Hardware component class (drivetrain, intake, outtake) that creates hardware devices
 */

package org.firstinspires.ftc.teamcode.Hardware;

public interface RobotHardware {
    void initHardware(boolean resetEncoders); // every Hardware component that creates hardware devices must have this method
}
