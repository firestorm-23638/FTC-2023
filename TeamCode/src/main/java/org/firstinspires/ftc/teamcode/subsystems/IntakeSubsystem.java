package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.BaseSubsystem;

public class IntakeSubsystem implements BaseSubsystem {
    private Gamepad gamepad;
    private HardwareMap hardwareMap;

    private Servo servo;

    public IntakeSubsystem(Gamepad gamepad, HardwareMap hardwareMap) {
        this.gamepad = gamepad;
        this.hardwareMap = hardwareMap;
    }

    public void init() {
        servo = this.hardwareMap.get(Servo.class, "intakeServo");

        servo.setPosition(0.6);
    }

    public void loop() {
        if (gamepad.right_bumper) {
            servo.setPosition(0);
        }
        else {
            servo.setPosition(0.6);
        }
    }
}


