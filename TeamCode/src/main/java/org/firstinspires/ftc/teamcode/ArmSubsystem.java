package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ClawMode;
import org.firstinspires.ftc.teamcode.ArmMode;

import org.firstinspires.ftc.teamcode.BaseSubsystem;


public class ArmSubsystem implements BaseSubsystem {
    private ClawMode clawPos;
    private ArmMode armPos;

    private final double armDropPos = 500;
    private final double clawStartPlacePos = 100;
    private double armEncoder;
    private double hardOutput = 0;

    private boolean clawPickupState = false;
    private boolean armRaisedState = false;

    private DcMotorEx arm1;
    private DcMotorEx arm2;
    private DcMotorEx intake;
    private Servo pivot;
    private Servo claw;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    private int clawStage = 1;

    public ArmSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

    public void configHardOutput(double val) {
        hardOutput = val;
    }

    public void configArmState(boolean clawPickupState, boolean armRaisedState) {
        this.clawPickupState = clawPickupState;
        this.armRaisedState = armRaisedState;
    }

    public void init() {
        arm1 = hardwareMap.get(DcMotorEx.class, "arm");
        arm2 = hardwareMap.get(DcMotorEx.class, "arm2");

        pivot = this.hardwareMap.get(Servo.class, "clawServo");
        claw = this.hardwareMap.get(Servo.class, "pivotServo");

        pivot.setPosition(0);
        claw.setPosition(0);

        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm2.setDirection(DcMotorSimple.Direction.REVERSE);

        arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private boolean setClaw(double pos) {
        claw.setPosition(pos);
        return (claw.getPosition() == pos);
    }

    private boolean setPivot(double pos) {
        pivot.setPosition(pos);
        return (pivot.getPosition() == pos);
    }

    private double averageArmPos() {
        return (arm1.getCurrentPosition() + (arm2.getCurrentPosition() * -1)) / 2;
    }

    public void loop() {
        telemetry.addData("Arm avg pos", averageArmPos());
        if (averageArmPos() > clawStartPlacePos) {
            pivot.setPosition(0.00055 * (averageArmPos() - 100));
        }
        else if (clawPickupState) {
            if (clawStage == 1) {
                if (setClaw(0)) {
                    clawStage ++;
                }
            }
            else if (clawStage == 2) {
                if (setPivot(.22)) {
                    clawStage ++;
                }
            }
            else if (clawStage == 3) {
                if (setClaw(.25)) {
                    clawStage ++;
                }
            }
            else {
                if (setPivot(0)) {
                    clawStage ++;
                }
            }
        }
        else {
            clawStage = 1;
        }

        arm1.setPower(this.hardOutput);
        arm2.setPower(this.hardOutput * -1);
    }
}
