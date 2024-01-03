package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.BaseSubsystem;

public class PlaneSubsystem implements BaseSubsystem {
    private Gamepad gamepad;

    private Servo planeServo;
    private Servo holdServo;

    private double tick = 0;

    public PlaneSubsystem(HardwareMap hardwareMap, Gamepad gamepad) {
        planeServo = hardwareMap.get(Servo.class, "planeServo");
        holdServo = hardwareMap.get(Servo.class, "holdServo");

        this.gamepad = gamepad;
    }

    public void init() {
        planeServo.setPosition(0.5);
        holdServo.setPosition(0.13);
    }

    public void loop() {
        if (this.gamepad.back) {
            holdServo.setPosition(0);
            if (this.gamepad.x) {
                planeServo.setPosition(0);
            }
        }
        else {
            holdServo.setPosition(0.13);
            planeServo.setPosition(0.5);
        }
    }
}
