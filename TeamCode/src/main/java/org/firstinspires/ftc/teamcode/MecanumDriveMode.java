package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.IMU;

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

        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        IMU gyro = this.hardwareMap.get(IMU.class, "imu");
        frontLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);

        double speedLimit = .3;
        double setAngle = 0;
        double angleOffset = -361;
        waitForStart();

        while (opModeIsActive()) {
            Orientation orientation = gyro.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            if (angleOffset == -361) {
                angleOffset = orientation.thirdAngle;
            }

            telemetry.addData("Gyro", orientation.thirdAngle);
            double frontLeftAmt =  speedLimit * (this.gamepad1.left_stick_y - this.gamepad1.left_stick_x) - this.gamepad1.right_stick_x * speedLimit;
            double frontRightAmt = speedLimit * (this.gamepad1.left_stick_y + this.gamepad1.left_stick_x) + this.gamepad1.right_stick_x * speedLimit;
            double backLeftAmt =   speedLimit * (this.gamepad1.left_stick_y + this.gamepad1.left_stick_x) - this.gamepad1.right_stick_x * speedLimit;
            double backRightAmt =  speedLimit * (this.gamepad1.left_stick_y - this.gamepad1.left_stick_x) + this.gamepad1.right_stick_x * speedLimit;

            if (this.gamepad1.right_bumper) {
                speedLimit = .6;
            }
            else {
                speedLimit = .3;
            }



            frontLeftMotor.setPower(frontLeftAmt);
            frontRightMotor.setPower(frontRightAmt);
            backLeftMotor.setPower(backLeftAmt);
            backRightMotor.setPower(backRightAmt);
            telemetry.update();
        }
    }
}
