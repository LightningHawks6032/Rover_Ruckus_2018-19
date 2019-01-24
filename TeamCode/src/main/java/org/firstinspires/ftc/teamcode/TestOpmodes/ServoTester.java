package org.firstinspires.ftc.teamcode.TestOpmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoTester {
    private OpMode teleOp;
    private Servo servo;
    private Gamepad gamepad;
    private double servoPos;

    public ServoTester(OpMode tele, Servo s, Gamepad g) {
        teleOp = tele;
        servo = s;
        gamepad = g;
        servoPos = 0; // DEFAULT
    }

    private boolean increaseTogglePressed = false;
    private boolean increaseToggleLastPressed = false;
    public void increase() {
        increaseTogglePressed = gamepad.a;

        if (increaseTogglePressed && !increaseToggleLastPressed) {
            servoPos += 0.1;
        }

        increaseToggleLastPressed = increaseTogglePressed;
    }

    private boolean decreaseTogglePressed = false;
    private boolean decreaseToggleLastPressed = false;
    public void decrease() {
        increaseTogglePressed = gamepad.y;

        if (decreaseTogglePressed && !decreaseToggleLastPressed) {
            servoPos -= 0.1;
        }

        decreaseToggleLastPressed = decreaseTogglePressed;
    }

    public void display() {
        teleOp.telemetry.addData("Actual Servo Position", servo.getPosition());
        teleOp.telemetry.addData("Set Servo Position", servo.getPosition());
        teleOp.telemetry.update();
    }

    public void run() {
        increase();
        decrease();
        servo.setPosition(servoPos);
        display();
    }
}
