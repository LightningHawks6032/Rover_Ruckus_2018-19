package org.firstinspires.ftc.teamcode.Robot1;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware.Robot1_Hardware;
import org.firstinspires.ftc.teamcode.Vision.Detectors.GoldAlignDetector;

public class Robot1_Auto {
    private Robot1_Hardware hardware;
    private GoldAlignDetector mineralDetector;

    // Constructor instantiates hardware and setups mineral detector
    public Robot1_Auto(Robot1_Hardware hardware) {
        this.hardware = hardware;
        mineralDetector = hardware.mineralDetector;
    }

    public void setupMineralDetector(HardwareMap hwMap) {
        mineralDetector.setupDetector(hwMap, 1);
    }

    // Turn until gold mineral is aligned with robot center
    public void turnToGold() throws InterruptedException {
        double turningPower;

        Thread.sleep(2000);
        if (mineralDetector.isFound()) {
            double startX = mineralDetector.getXPosition();

            while (!mineralDetector.getAligned()) {
                //turn towards gold (robot center x is less than x position = turn right)
                turningPower = Math.abs(mineralDetector.getXPosition() - mineralDetector.getRobotCenterX()) / (startX - mineralDetector.getRobotCenterX()) * 0.2 + 0.1;

                hardware.drivetrain.setPowers(turningPower, -turningPower*0.5, 0);
            }
            hardware.drivetrain.setPowers(0, 0, 0);

        } else {
            //do something to get it into vision
        }
    }

}
