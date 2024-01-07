package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.BaseSubsystem;

public class ClimbSubsystem implements BaseSubsystem {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private Gamepad gamepad;

    private DcMotorEx climbMotor;
    private DcMotorEx climbMotor2;

    public ClimbSubsystem(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.gamepad = gamepad;
    }

    public void init() {
        climbMotor = this.hardwareMap.get(DcMotorEx.class, "climb");
        climbMotor2 = this.hardwareMap.get(DcMotorEx.class, "climb2");

        climbMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        climbMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climbMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void loop() {
        if (gamepad.dpad_down) {
            climbMotor.setPower(-1);
            climbMotor2.setPower(.85);
        }
        else if (gamepad.dpad_up) {
            climbMotor.setPower(1);
            climbMotor2.setPower(-.85);
        }
        else {
            climbMotor.setPower(0);
            climbMotor2.setPower(0);
        }
    }
}
