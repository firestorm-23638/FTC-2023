package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.BaseSubsystem;

public class IntakeSubsystem implements BaseSubsystem {
    private Gamepad gamepad;
    private HardwareMap hardwareMap;

    private DcMotorEx motor;

    public IntakeSubsystem(Gamepad gamepad, HardwareMap hardwareMap) {
        this.gamepad = gamepad;
        this.hardwareMap = hardwareMap;
    }

    public void init() {
        this.motor = this.hardwareMap.get(DcMotorEx.class, "intake");
        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void loop() {
        if (gamepad.right_bumper) {
            motor.setPower(0.5);
        }
        else if (gamepad.left_bumper) {
            motor.setPower(-0.5);
        }
        else {
            motor.setPower(0);
        }
    }
}


