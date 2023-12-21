package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class ArmSubsystem implements BaseSubsystem {

    private boolean clawPickupState = false;
    private boolean pivotPickupState = false;
    private boolean twoControllerMode = false;
    private double armOutput = 0;

    private DcMotorEx arm;
    private Servo pivot;
    private Servo claw;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private Gamepad gamepad;

    private int clawStage = 1;

    private boolean withinDeadband(double n, double r, double d) {
        n -= r;
        return (n < d) && (n > -d);
    }

    public ArmSubsystem(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad, boolean twoControllerMode) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.gamepad = gamepad;
        this.twoControllerMode = twoControllerMode;

        pivot = this.hardwareMap.get(Servo.class, "clawServo");
        claw = this.hardwareMap.get(Servo.class, "pivotServo");
        arm = this.hardwareMap.get(DcMotorEx.class, "arm");
    }

    public void init() {

        pivot.setPosition(0);
        claw.setPosition(0);

        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setArmPos(double pos, boolean blocking) {
        arm.setPower((arm.getCurrentPosition() - pos) * -1.001);
        while (blocking) {
            if (withinDeadband(arm.getCurrentPosition(), pos, 10)) {
                arm.setPower(0);
                return;
            }
        }
    }

    public void setLeftClaw(double pos) {

    }

    public void setRightClaw(double pos) {

    }

    public void setPivot(double pos) {
        pivot.setPosition(pos);
    }

    public void loop() {
        arm.setPower(this.gamepad.right_trigger - this.gamepad.left_trigger);

        if (this.twoControllerMode) {
            if (this.gamepad.a) {
                setPivot(0.32);
            }
            else {
                setPivot(0);
            }
            if (this.gamepad.left_bumper) {
                setLeftClaw(.3);
            }
            else {
                setLeftClaw(0);
            }
            if (this.gamepad.right_bumper) {
                setRightClaw(.3);
            }
            else {
                setRightClaw(0);
            }
        }
        else {

        }
    }
}