import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cAddr;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// Just a copy-paste of the sample opmode

@TeleOp(name = "Sensor: MR range sensor", group = "Sensor")
public class RangeSensorTest extends LinearOpMode {
    ModernRoboticsI2cRangeSensor rangeSensor;

    public void runOpMode() {

        // get a reference to our compass
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rs");
        rangeSensor.setI2cAddress(new I2cAddr(0x2c));

        // wait for the start button to be pressed
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("raw ultrasonic", rangeSensor.rawUltrasonic());
            telemetry.addData("raw optical", rangeSensor.rawOptical());
            telemetry.addData("cm optical", "%.2f cm", rangeSensor.cmOptical());
            telemetry.addData("cm", "%.2f cm", rangeSensor.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }
}
