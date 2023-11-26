package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

@TeleOp
public class MecanumDriveMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotorEx intakeMotor = this.hardwareMap.get(DcMotorEx.class, "intake");
        DcMotorEx climbMotor = this.hardwareMap.get(DcMotorEx.class, "climb");

        climbMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Servo pivot = this.hardwareMap.get(Servo.class, "clawServo");
        Servo claw = this.hardwareMap.get(Servo.class, "pivotServo");
        Servo plane = this.hardwareMap.get(Servo.class, "planeServo");

        DrivetrainSubsystem drivetrain = new DrivetrainSubsystem(this.hardwareMap, telemetry);
        ArmSubsystem arm = new ArmSubsystem(this.hardwareMap, telemetry);

        double speedLimit = 0.5;

        drivetrain.init();
        arm.init();

        waitForStart();

        while (opModeIsActive()) {
            drivetrain.setToValues(this.gamepad1.left_stick_x, this.gamepad1.left_stick_y, this.gamepad1.right_stick_x, 0);
            arm.configArmState(this.gamepad1.a, this.gamepad1.b, this.gamepad1.left_trigger - this.gamepad1.right_trigger);

            if (this.gamepad1.right_bumper) {
                intakeMotor.setPower(0.8);
            }
            else if (this.gamepad1.left_bumper) {
                intakeMotor.setPower(-0.8);
            }
            else {
                intakeMotor.setPower(0);
            }

            if (this.gamepad1.back) {
                plane.setPosition(0);
            }
            else {
                plane.setPosition(0.5);
            }

            telemetry.addData("Climb motor encoder", climbMotor.getCurrentPosition());
            if (this.gamepad1.dpad_down) {
                climbMotor.setPower(1);
            }
            else if (this.gamepad1.dpad_up) {
                climbMotor.setPower(-1);
            }
            else {
                climbMotor.setPower(0);
            }


            arm.loop();
            drivetrain.loop();

            telemetry.update();
        }
    }
}