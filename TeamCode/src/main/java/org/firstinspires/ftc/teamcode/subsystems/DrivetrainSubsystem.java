package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.Vector;
import org.firstinspires.ftc.teamcode.subsystems.BaseSubsystem;

public class DrivetrainSubsystem implements BaseSubsystem {

    private DcMotorEx frontLeftMotor;
    private DcMotorEx frontRightMotor;
    private DcMotorEx backLeftMotor;
    private DcMotorEx backRightMotor;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private Gamepad gamepad;

    private final boolean isInverted = true;

    public DrivetrainSubsystem(HardwareMap map, Telemetry telemetry, Gamepad gamepad) {
        this.hardwareMap = map;
        this.telemetry = telemetry;
        this.gamepad = gamepad;

        this.frontLeftMotor = this.hardwareMap.get(DcMotorEx.class, "frontLeft");
        this.frontRightMotor = this.hardwareMap.get(DcMotorEx.class, "frontRight");
        this.backLeftMotor = this.hardwareMap.get(DcMotorEx.class, "backLeft");
        this.backRightMotor = this.hardwareMap.get(DcMotorEx.class, "backRight");
    }

    private double coterminalAngle(double ang) {
        while (ang > 360) {
            ang -= 360;
        }
        while (ang < 0) {
            ang += 360;
        }
        return ang;
    }

    private double frontLeftAmt = 0;
    private double frontRightAmt = 0;
    private double backLeftAmt = 0;
    private double backRightAmt = 0;

    public double speedLimit = 0.4;

    public Vector applyVector = new Vector();

    public void setToValues(double x, double y, double turn, double gyroOffset) {
        /*this.applyVector.x = x;
        this.applyVector.y = y;
        this.applyVector.setAngle(coterminalAngle(this.applyVector.getAngle() + gyroOffset));//coterminalAngle(this.applyVector.getAngle() - gyroOffset));
        telemetry.addData("drive vector x", Math.cos(Math.toRadians(coterminalAngle(this.applyVector.getAngle() + gyroOffset))));
        telemetry.addData("drive vector y", Math.sin(Math.toRadians(coterminalAngle(this.applyVector.getAngle() + gyroOffset))));
        telemetry.addData("vector angle", coterminalAngle(this.applyVector.getAngle() + gyroOffset));
        telemetry.addData("vector mag", Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)));
        telemetry.addData("drive vector Real x", this.applyVector.x);
        telemetry.addData("drive vector Real y", this.applyVector.y);*/

        this.frontLeftAmt =  speedLimit * (y - x) - turn * speedLimit;
        this.frontRightAmt = speedLimit * (y + x) + turn * speedLimit;
        this.backLeftAmt =   speedLimit * (y + x) - turn * speedLimit;
        this.backRightAmt =  speedLimit * (y - x) + turn * speedLimit;
    }

    public void turn(double perc) {
        this.frontLeftAmt = perc;
        this.frontRightAmt = -perc;
        this.backLeftAmt = perc;
        this.backRightAmt = -perc;
    }

    public void forward(double perc) {
        this.frontLeftAmt = perc;
        this.frontRightAmt = perc;
        this.backLeftAmt = perc;
        this.backRightAmt = perc;
    }

    public void zero() {
        this.frontLeftAmt = 0;
        this.frontRightAmt = 0;
        this.backRightAmt = 0;
        this.backLeftAmt = 0;
    }

    public void init() {
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public void loop() {
        telemetry.addData("joystick pos", Math.abs(this.gamepad.left_stick_x));
        if (Math.abs(this.gamepad.left_stick_x) < 0.7) {
            this.frontLeftAmt =  speedLimit * (this.gamepad.left_stick_y - (this.gamepad.left_stick_x * 1.6)) + this.gamepad.right_stick_x * speedLimit;   // 1.7
            this.frontRightAmt = speedLimit * (this.gamepad.left_stick_y + (this.gamepad.left_stick_x * 1.6)) - this.gamepad.right_stick_x * speedLimit;   // 1.7
            this.backLeftAmt =   speedLimit * (this.gamepad.left_stick_y + (this.gamepad.left_stick_x * 1.85)) + this.gamepad.right_stick_x * speedLimit;   // 1.8
            this.backRightAmt =  speedLimit * (this.gamepad.left_stick_y - (this.gamepad.left_stick_x * 1.85)) - this.gamepad.right_stick_x * speedLimit;   // 1.8
        }
        else {
            this.frontLeftAmt =  speedLimit * (this.gamepad.left_stick_y - (this.gamepad.left_stick_x * 1.6)) + this.gamepad.right_stick_x * speedLimit;   // 1.7
            this.frontRightAmt = speedLimit * (this.gamepad.left_stick_y + (this.gamepad.left_stick_x * 1.6)) - this.gamepad.right_stick_x * speedLimit;   // 1.7
            this.backLeftAmt =   speedLimit * (this.gamepad.left_stick_y + (this.gamepad.left_stick_x * 1.75)) + this.gamepad.right_stick_x * speedLimit;   // 1.8
            this.backRightAmt =  speedLimit * (this.gamepad.left_stick_y - (this.gamepad.left_stick_x * 1.75)) - this.gamepad.right_stick_x * speedLimit;   // 1.8
        }

        if (isInverted) {
            this.frontLeftAmt *= -1;
            this.frontRightAmt *= -1;
            this.backLeftAmt *= -1;
            this.backRightAmt *= -1;
        }

        frontLeftMotor.setPower(this.frontLeftAmt);
        frontRightMotor.setPower(this.frontRightAmt);
        backLeftMotor.setPower(this.backLeftAmt);
        backRightMotor.setPower(this.backRightAmt);
    }
}
