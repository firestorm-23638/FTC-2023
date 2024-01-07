package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClimbSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PlaneSubsystem;

@TeleOp
public class TwoControllerDriveMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotorEx intakeMotor = this.hardwareMap.get(DcMotorEx.class, "intake");

        DrivetrainSubsystem drivetrain = new DrivetrainSubsystem(this.hardwareMap, telemetry, this.gamepad1);
        ArmSubsystem arm = new ArmSubsystem(this.hardwareMap, telemetry, this.gamepad2, true);
        PlaneSubsystem plane = new PlaneSubsystem(this.hardwareMap, this.gamepad2);
        ClimbSubsystem climb = new ClimbSubsystem(this.hardwareMap, telemetry, this.gamepad2);

        drivetrain.init();
        arm.init();
        plane.init();
        climb.init();

        waitForStart();

        while (opModeIsActive()) {
            if (this.gamepad1.right_bumper) {
                intakeMotor.setPower(0.2);
            }
            else {
                intakeMotor.setPower(this.gamepad1.right_trigger - this.gamepad1.left_trigger);
            }

            arm.loop();
            drivetrain.loop();
            plane.loop();
            climb.loop();

            telemetry.update();
        }
    }
}