package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.ClawMode;
import org.firstinspires.ftc.teamcode.ArmMode;

import org.firstinspires.ftc.teamcode.BaseSubsystem;


public class ArmSubsystem implements BaseSubsystem {
    private ClawMode clawPos;
    private ArmMode armPos;
    private double armEncoder;

    public DcMotorEx arm1;
    public DcMotorEx arm2;
    public DcMotorEx intake;
    public Servo pivot;
    public Servo claw;

    public void init() {
        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm2.setDirection(DcMotorSimple.Direction.REVERSE);

        arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void loop() {
        
        /*if (armPos == ArmMode.HOME) {
            arm1.setTargetPosition(0);
        }
        else if (armPos == ArmMode.PLACE) {
            arm1.setTargetPosition(500);
        }*/
    }
}
