package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp
public class MecanumDriveMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx frontLeftMotor = this.hardwareMap.get(DcMotorEx.class, "frontLeft");
        DcMotorEx frontRightMotor = this.hardwareMap.get(DcMotorEx.class, "frontRight");
        DcMotorEx backLeftMotor = this.hardwareMap.get(DcMotorEx.class, "backLeft");
        DcMotorEx backRightMotor = this.hardwareMap.get(DcMotorEx.class, "backRight");

        DcMotorEx intakeMotor = this.hardwareMap.get(DcMotorEx.class, "intake");
        DcMotorEx armMotor = this.hardwareMap.get(DcMotorEx.class, "arm");

        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        Servo pivot = this.hardwareMap.get(Servo.class, "clawServo");
        Servo claw = this.hardwareMap.get(Servo.class, "pivotServo");

        IMU gyro = this.hardwareMap.get(IMU.class, "imu");
        frontLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);

        double speedLimit = .5;
        double setAngle = 0;
        double angleOffset = -361;
        boolean servosZeroed = false;
        waitForStart();

        while (opModeIsActive()) {
            Orientation orientation = gyro.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            if (angleOffset == -361) {
                angleOffset = orientation.thirdAngle;
            }

            telemetry.addData("Gyro", orientation.thirdAngle);
            if (!servosZeroed) {
                claw.setPosition(0);
                pivot.setPosition(0);
                servosZeroed = true;

            }
            telemetry.addData("Arm encoder", armMotor.getCurrentPosition());

            double frontLeftAmt =  speedLimit * (this.gamepad1.left_stick_y - this.gamepad1.left_stick_x) - this.gamepad1.right_stick_x * speedLimit;
            double frontRightAmt = speedLimit * (this.gamepad1.left_stick_y + this.gamepad1.left_stick_x) + this.gamepad1.right_stick_x * speedLimit;
            double backLeftAmt =   speedLimit * (this.gamepad1.left_stick_y + this.gamepad1.left_stick_x) - this.gamepad1.right_stick_x * speedLimit;
            double backRightAmt =  speedLimit * (this.gamepad1.left_stick_y - this.gamepad1.left_stick_x) + this.gamepad1.right_stick_x * speedLimit;
            armMotor.setPositionPIDFCoefficients(.0015);
            if (this.gamepad1.right_bumper) {
                //speedLimit = .6;
                intakeMotor.setPower(.25);
            }
            else if (this.gamepad1.left_bumper) {
                intakeMotor.setPower(-.25);
            }
            else {
                intakeMotor.setPower(0);
            }

            if (this.gamepad1.a) {
                claw.setPosition(0);
            }
            else {
                claw.setPosition(.25);
            }
            armMotor.setPower((this.gamepad1.right_trigger - this.gamepad1.left_trigger) * .7);


            if (this.gamepad1.b) {
                pivot.setPosition(.22);
            }
            else {
                pivot.setPosition(0);
            }

            frontLeftMotor.setPower(frontLeftAmt);
            frontRightMotor.setPower(frontRightAmt);
            backLeftMotor.setPower(backLeftAmt);
            backRightMotor.setPower(backRightAmt);
            telemetry.update();
        }
    }
}
