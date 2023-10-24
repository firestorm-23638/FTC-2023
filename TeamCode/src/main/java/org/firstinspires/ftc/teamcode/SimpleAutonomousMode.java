package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class SimpleAutonomousMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx frontLeftMotor = this.hardwareMap.get(DcMotorEx.class, "frontLeft");
        DcMotorEx frontRightMotor = this.hardwareMap.get(DcMotorEx.class, "frontRight");
        DcMotorEx backLeftMotor = this.hardwareMap.get(DcMotorEx.class, "backLeft");
        DcMotorEx backRightMotor = this.hardwareMap.get(DcMotorEx.class, "backRight");

        frontLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);

        waitForStart();
        double begTime = 0;
        while (opModeIsActive()) {
            begTime ++;
            if (begTime < 25000) {
                frontLeftMotor.setPower(.25);
                frontRightMotor.setPower(.25);
                backLeftMotor.setPower(.25);
                backRightMotor.setPower(.25);
            }
            else {
                frontLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backLeftMotor.setPower(0);
                backRightMotor.setPower(0);
            }

        }
    }
}