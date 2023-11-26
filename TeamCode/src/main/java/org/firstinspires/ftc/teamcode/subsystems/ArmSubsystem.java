package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class ArmSubsystem implements BaseSubsystem {

    private boolean clawPickupState = false;
    private boolean pivotPickupState = false;
    private double armOutput = 0;

    private DcMotorEx arm;
    private Servo pivot;
    private Servo claw;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    private int clawStage = 1;

    private boolean withinDeadband(double n, double r, double d) {
        n -= r;
        return (n < d) && (n > -d);
    }

    public ArmSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        pivot = this.hardwareMap.get(Servo.class, "clawServo");
        claw = this.hardwareMap.get(Servo.class, "pivotServo");
        arm = this.hardwareMap.get(DcMotorEx.class, "arm");
    }

    public void init() {

        pivot.setPosition(0);
        claw.setPosition(0.3);

        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setArmPos(double pos, boolean blocking) {
        arm.setPower((arm.getCurrentPosition() - pos) * -1.001);
        while (blocking) {
            if (withinDeadband(arm.getCurrentPosition(), pos, 10)) {
                return;
            }
        }
    }

    public void configArmState(boolean clawPickupState, boolean pivotPickupState, double armOutput) {
        this.clawPickupState = clawPickupState;
        this.pivotPickupState = pivotPickupState;
        this.armOutput = armOutput;
    }

    public void setClaw(double pos) {
        claw.setPosition(pos);
    }

    public void setPivot(double pos) {
        pivot.setPosition(pos);
    }

    public void loop() {
        arm.setPower(this.armOutput);

        if (clawPickupState) {
            setClaw(0.3);
        }
        else {
            setClaw(0);
        }
        if (pivotPickupState) {
            setPivot(0.36);
        }
        else {
            setPivot(0);
        }
    }
}