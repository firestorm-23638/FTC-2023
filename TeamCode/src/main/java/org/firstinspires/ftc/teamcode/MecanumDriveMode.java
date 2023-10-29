package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ArmSubsystem;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.DrivetrainSubsystem;

@TeleOp
public class MecanumDriveMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotorEx intakeMotor = this.hardwareMap.get(DcMotorEx.class, "intake");

        Servo pivot = this.hardwareMap.get(Servo.class, "clawServo");
        Servo claw = this.hardwareMap.get(Servo.class, "pivotServo");

        IMU gyro = this.hardwareMap.get(IMU.class, "imu");

        DrivetrainSubsystem drivetrain = new DrivetrainSubsystem(this.hardwareMap, telemetry);
        ArmSubsystem arm = new ArmSubsystem(this.hardwareMap, telemetry);

        double speedLimit = .5;
        double angleOffset = -361;

        drivetrain.init();
        arm.init();
        waitForStart();

        while (opModeIsActive()) {
            Orientation orientation = gyro.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

            if (angleOffset == -361 || this.gamepad1.x) {
                angleOffset = orientation.thirdAngle;
            }

            drivetrain.setToValues(this.gamepad1.left_stick_x, this.gamepad1.left_stick_y, this.gamepad1.right_stick_x, orientation.thirdAngle - angleOffset);

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

            arm.configHardOutput(this.gamepad1.right_trigger - this.gamepad1.left_trigger);
            arm.configArmState(this.gamepad1.a, this.gamepad1.b);

            arm.loop();
            drivetrain.loop();

            telemetry.update();
        }
    }
}