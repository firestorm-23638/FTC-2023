package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PlaneSubsystem;

@TeleOp
public class TwoControllerDriveMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotorEx intakeMotor = this.hardwareMap.get(DcMotorEx.class, "intake");

        DcMotorEx climbMotor = this.hardwareMap.get(DcMotorEx.class, "climb");

        climbMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DrivetrainSubsystem drivetrain = new DrivetrainSubsystem(this.hardwareMap, telemetry, this.gamepad1);
        ArmSubsystem arm = new ArmSubsystem(this.hardwareMap, telemetry, this.gamepad2, true);
        PlaneSubsystem plane = new PlaneSubsystem(this.hardwareMap, this.gamepad2);

        double speedLimit = 0.5;

        drivetrain.init();
        arm.init();
        plane.init();

        waitForStart();

        while (opModeIsActive()) {

            intakeMotor.setPower(this.gamepad1.left_trigger - this.gamepad1.right_trigger);

           climbMotor.setPower(this.gamepad2.left_stick_y);

            arm.loop();
            drivetrain.loop();
            plane.loop();

            telemetry.update();
        }
    }
}