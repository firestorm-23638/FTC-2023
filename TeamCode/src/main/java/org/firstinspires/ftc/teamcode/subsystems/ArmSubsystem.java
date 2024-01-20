package org.firstinspires.ftc.teamcode.subsystems;

import android.app.ListActivity;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class ArmSubsystem implements BaseSubsystem {
    private boolean twoControllerMode = false;
    private boolean isAtGoal = true;
    private boolean armGarageButtonState = false;

    private double armGarageTarget = 0;

    private DcMotorEx arm;
    private Servo pivot;
    private Servo leftClaw;
    private Servo rightClaw;
    private TouchSensor armSwitch;

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
        armSwitch = this.hardwareMap.get(TouchSensor.class, "armSwitch");
    }

    public void init() {
        pivot.setPosition(0.25);
        leftClaw.setPosition(0.18);
        rightClaw.setPosition(0);

        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double calculatePIDOutput(double armPos, double t) {
        //double kf = 0.05;

        //if (armPos > 330) {
        //    kf *= -1;
        //}

        double target = ((armPos - t) * -0.022);

        if (target < -1) {
            target = -1;
        }
        else if (target > 1) {
            target = 1;
        }

        return target;
    }

    public boolean setArmPos(double pos, boolean blocking) {
        arm.setPower(calculatePIDOutput(arm.getCurrentPosition(), pos));
        while (blocking) {

            arm.setPower(calculatePIDOutput(arm.getCurrentPosition(), pos));
            if (withinDeadband(arm.getCurrentPosition(), pos, 7)) {
                arm.setPower(0);
                return true;
            }
        }
        return withinDeadband(arm.getCurrentPosition(), pos, 7);
    }

    public boolean armGarageButtonPressed() {
        if (!this.gamepad.a) {
            armGarageButtonState = true;
        }
        else if (armGarageButtonState) {
            armGarageButtonState = false;
            return true;
        }
        return false;
    }

    public void setRawArm(double perc) {
        arm.setPower(perc);
    }

    public void setLeftClaw(double pos) {
        leftClaw.setPosition(pos);
    }

    public void setRightClaw(double pos) {
        rightClaw.setPosition(pos);
    }

    public void setPivot(double pos) {pivot.setPosition(pos);}

    public void openLeftClaw() {
        setLeftClaw(0.07);
    }

    public void closeLeftClaw() {
        setLeftClaw(0.25);
    }

    public void openRightClaw() {
        setRightClaw(0.1);
    }

    public void closeRightClaw() {
        setRightClaw(0);
    }

    public void loop() {
        telemetry.addData("is at goal", isAtGoal);

        if (armGarageButtonPressed()) {
            isAtGoal = false;
            if (armGarageTarget == 0) {
                armGarageTarget = 950;
            }
            else {
                armGarageTarget = 0;
            }
        }

        if (!isAtGoal) {
            isAtGoal = setArmPos(armGarageTarget, false);
        }


        if (armSwitch.isPressed()) {
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (isAtGoal) {
                arm.setPower(this.gamepad.right_trigger);
            }

        }
        else {
            if (isAtGoal) {
                arm.setPower(this.gamepad.right_trigger - this.gamepad.left_trigger);
            }
        }

        if (this.gamepad.right_stick_y != 0) {
            setPivot((this.gamepad.right_stick_y + 1) * .2);
        }
        else {
            if (arm.getCurrentPosition() > 450) {
                if (((arm.getCurrentPosition() - 450) * 0.00076) - 0.13 == 0) {
                    setPivot(0);
                }
                else {
                    setPivot(((arm.getCurrentPosition() - 450) * 0.00076) - 0.13);
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
                openLeftClaw();
            }
            else {
                closeLeftClaw();
            }
            if (this.gamepad.left_bumper) {
                openRightClaw();
            }
            else {
                closeRightClaw();
            }
        }
        else {
            if (this.gamepad.a) {
                openLeftClaw();
                openRightClaw();
            }
            else {
                closeLeftClaw();
                closeRightClaw();
            }
        }
    }
}