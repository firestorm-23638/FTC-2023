package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DrivetrainSubsystem implements BaseSubsystem {
    private DcMotorEx backLeftMotor;
    private DcMotorEx backRightMotor;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private Gamepad gamepad;

    public DrivetrainSubsystem(HardwareMap map, Telemetry telemetry, Gamepad gamepad) {
        this.hardwareMap = map;
        this.telemetry = telemetry;
        this.gamepad = gamepad;

        this.backLeftMotor = this.hardwareMap.get(DcMotorEx.class, "backLeft");
        this.backRightMotor = this.hardwareMap.get(DcMotorEx.class, "backRight");
    }

    public double speedLimit = 0.6;

    public void init() {
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public void loop() {
        /*backLeftMotor.setPower(gamepad.left_stick_y * speedLimit);
        backRightMotor.setPower(gamepad.right_stick_y * speedLimit);*/
        backLeftMotor.setPower(gamepad.left_stick_y * speedLimit - (gamepad.right_stick_x * speedLimit * .7));
        backRightMotor.setPower(gamepad.left_stick_y * speedLimit + (gamepad.right_stick_x * speedLimit));
    }
}
