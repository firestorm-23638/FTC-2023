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
    private boolean isJoystickOverride = false;
    private double armOutput = 0;

    private DcMotorEx arm;
    private Servo pivot;
    private Servo leftClaw;
    private Servo rightClaw;

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

        pivot = this.hardwareMap.get(Servo.class, "pivot");
        leftClaw = this.hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = this.hardwareMap.get(Servo.class, "rightClaw");
        arm = this.hardwareMap.get(DcMotorEx.class, "arm");
    }

    public void init() {
        pivot.setPosition(0.25);
        leftClaw.setPosition(0.18);
        rightClaw.setPosition(0);

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
        leftClaw.setPosition(pos);
    }

    public void setRightClaw(double pos) {
        rightClaw.setPosition(pos);
    }

    public void setPivot(double pos) {pivot.setPosition(pos);}

    public void loop() {
        arm.setPower(this.gamepad.right_trigger - this.gamepad.left_trigger);
        telemetry.addData("claw new pos", ((arm.getCurrentPosition() - 550) * 0.00065) - 0.13);
        if (this.gamepad.right_stick_y != 0) {
            setPivot((this.gamepad.right_stick_y + 1) * .2);
        }
        else {
            if (arm.getCurrentPosition() > 550) {
                if (((arm.getCurrentPosition() - 550) * 0.00055) - 0.13 == 0) {
                    setPivot(0);
                }
                else {
                    setPivot(((arm.getCurrentPosition() - 550) * 0.00055) - 0.13);
                }
            }
            else {
                if (this.gamepad.b) {
                    setPivot(0.46);
                }
                else {
                    setPivot(0.25);
                }
            }
        }

        if (this.twoControllerMode) {
            if (this.gamepad.right_bumper) {
                setLeftClaw(0.07);
            }
            else {
                setLeftClaw(0.18);
            }
            if (this.gamepad.left_bumper) {
                setRightClaw(0.1);
            }
            else {
                setRightClaw(0);
            }
        }
        else {
            if (this.gamepad.a) {
                setLeftClaw(0.1);
                setRightClaw(0.12);
            }
            else {
                setLeftClaw(0.18);
                setRightClaw(0);
            }
        }
    }
}