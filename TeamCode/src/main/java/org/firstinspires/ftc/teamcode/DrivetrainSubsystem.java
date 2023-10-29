package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.teamcode.BaseSubsystem;

public class DrivetrainSubsystem implements BaseSubsystem {
    private DcMotorEx frontLeftMotor;
    private DcMotorEx frontRightMotor;
    private DcMotorEx backLeftMotor;
    private DcMotorEx backRightMotor;
    private HardwareMap hardwareMap;

    public DrivetrainSubsystem(HardwareMap map) {
        this.hardwareMap = map;

        this.frontLeftMotor = this.hardwareMap.get(DcMotorEx.class, "frontLeft");
        this.frontRightMotor = this.hardwareMap.get(DcMotorEx.class, "frontRight");
        this.backLeftMotor = this.hardwareMap.get(DcMotorEx.class, "backLeft");
        this.backRightMotor = this.hardwareMap.get(DcMotorEx.class, "backRight");
    }

    public double frontLeftAmt = 0;
    public double frontRightAmt = 0;
    public double backLeftAmt = 0;
    public double backRightAmt = 0;
    public double speedLimit = .5;

    public void setToValues(double x, double y, double turn, double gyroOffset) {
        this.frontLeftAmt =  speedLimit * (y - x) - turn * speedLimit;
        this.frontRightAmt = speedLimit * (y + x) + turn * speedLimit;
        this.backLeftAmt =   speedLimit * (y + x) - turn * speedLimit;
        this.backRightAmt =  speedLimit * (y - x) + turn * speedLimit;


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
