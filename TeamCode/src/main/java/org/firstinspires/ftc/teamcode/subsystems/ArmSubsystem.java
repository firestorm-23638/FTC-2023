package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.subsystems.BaseSubsystem;


public class ArmSubsystem implements BaseSubsystem {

    private final double armDropPos = 500;
    private final double clawStartPlacePos = 100;
    private double armEncoder = 0;
    private double hardOutput = 0;

    private boolean clawPickupState = false;
    private boolean pivotPickupState = false;
    private boolean armRaisedState = false;

    //private DcMotorEx arm1;
    //private DcMotorEx arm2;
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

    public void configArmState(boolean clawPickupState, boolean pivotPickupState, double armEncoderPos) {
        this.clawPickupState = clawPickupState;
        this.pivotPickupState = pivotPickupState;
        this.armEncoder = armEncoderPos;
    }

    public void init() {


        pivot = this.hardwareMap.get(Servo.class, "clawServo");
        claw = this.hardwareMap.get(Servo.class, "pivotServo");

        pivot.setPosition(0);
        claw.setPosition(.3);

        //arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //arm2.setDirection(DcMotorSimple.Direction.REVERSE);

        //arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private boolean setClaw(double pos) {
        claw.setPosition(pos);
        return (claw.getPosition() - pos > -0.05 && claw.getPosition() - pos < 0.05);
    }

    private boolean setPivot(double pos) {
        pivot.setPosition(pos);
        return (pivot.getPosition() - pos > -0.05 && pivot.getPosition() - pos < 0.05);
    }

    public void loop() {
        if (pivotPickupState) {
            setPivot(.2);
        }
        else {
            setPivot(0);
        }
        if (clawPickupState) {
            setClaw(0);
        }
        else {
            setClaw(.3);
        }

        //if (armEncoder > clawStartPlacePos) {
           //pivot.setPosition(0.00055 * (armEncoder - 100));
        //}
        /*else if (clawPickupState) {
            if (clawStage == 1) {
                if (setClaw(0)) {
                    clawStage ++;
                }
            }
            else if (clawStage == 2) {
                if (setPivot(.2)) {
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
            setClaw(.25);
            setPivot(0);
        }*/
    }
}
