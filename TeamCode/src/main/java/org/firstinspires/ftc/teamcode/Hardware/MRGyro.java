package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.GyroSensor;

import java.util.ArrayList;

public class MRGyro {
    private GyroSensor gyroSensor; // Hardware Device Object
    public MRGyro(GyroSensor gyro, boolean calibrate) {
        gyroSensor = gyro;

        if (calibrate)
            calibrate();
        zero();
    }

    public void calibrate() {
        gyroSensor.calibrate();
        while (gyroSensor.isCalibrating()) {
            // WAIT - Gyro Sensor is Calibrating
        }
    }

    public void zero() {
        gyroSensor.resetZAxisIntegrator();
    }

    // Returns heading -- adjusted for the fact that its reading backwards values
    public int getHeading() {
        return 360 - gyroSensor.getHeading();
    }

    // Returns the angle the robot has turned from the origin, negative for anything left and positive for anything right
    public int getAngle() {
        return convertToAngleFromOrigin(getHeading());
    }

    // We convert from gyro heading to angle turned from origin in this class, so this helper method makes things easier to read
    private int convertToAngleFromOrigin(int heading) {
        if (heading > 180) {
            return -(360 - heading);
        } else
            return heading;
    }

    public static double convertToDegrees(double radians) {
        return (radians * 180 / Math.PI);
    }
    public static double convertToRadians(double degrees) { return degrees * Math.PI / 180; }
}
