package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.Vector;
import org.firstinspires.ftc.teamcode.BaseSubsystem;

public class DrivetrainSubsystem implements BaseSubsystem {

    private DcMotorEx frontLeftMotor;
    private DcMotorEx frontRightMotor;
    private DcMotorEx backLeftMotor;
    private DcMotorEx backRightMotor;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    public DrivetrainSubsystem(HardwareMap map, Telemetry telemetry) {
        this.hardwareMap = map;
        this.telemetry = telemetry;

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
    public double speedLimit = .5;

    public Vector applyVector = new Vector();

    public void setToValues(double x, double y, double turn, double gyroOffset) {
        this.applyVector.x = x;
        this.applyVector.y = y;

        this.applyVector.setAngle(coterminalAngle(this.applyVector.getAngle() + gyroOffset));//coterminalAngle(this.applyVector.getAngle() - gyroOffset));

        this.frontLeftAmt =  speedLimit * (this.applyVector.y - this.applyVector.x) - turn * speedLimit;
        this.frontRightAmt = speedLimit * (this.applyVector.y + this.applyVector.x) + turn * speedLimit;
        this.backLeftAmt =   speedLimit * (this.applyVector.y + this.applyVector.x) - turn * speedLimit;
        this.backRightAmt =  speedLimit * (this.applyVector.y - this.applyVector.x) + turn * speedLimit;

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
        frontLeftMotor.setPower(this.frontLeftAmt);
        frontRightMotor.setPower(this.frontRightAmt);
        backLeftMotor.setPower(this.backLeftAmt);
        backRightMotor.setPower(this.backRightAmt);
    }
}
