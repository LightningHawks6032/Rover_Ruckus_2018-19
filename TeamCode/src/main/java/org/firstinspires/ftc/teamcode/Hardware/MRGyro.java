package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.GyroSensor;

public class MRGyro {
    private GyroSensor gyroSensor;

    public MRGyro(GyroSensor gyro) {
        gyroSensor = gyro;
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

    public int getAngle() {
        return 360 - gyroSensor.getHeading();
    }


}
