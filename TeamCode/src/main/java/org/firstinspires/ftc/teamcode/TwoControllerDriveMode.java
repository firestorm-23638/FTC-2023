package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PlaneSubsystem;

@TeleOp
public class TwoControllerDriveMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DrivetrainSubsystem drivetrain = new DrivetrainSubsystem(this.hardwareMap, telemetry, this.gamepad1);
        ArmSubsystem arm = new ArmSubsystem(this.hardwareMap, telemetry, this.gamepad2, true);
        PlaneSubsystem plane = new PlaneSubsystem(this.hardwareMap, this.gamepad2);
        IntakeSubsystem intake = new IntakeSubsystem(this.gamepad1, this.hardwareMap);

        drivetrain.init();
        arm.init();
        plane.init();
        intake.init();

        waitForStart();

        while (opModeIsActive()) {
            arm.setCurrentTime(time);

            intake.loop();
            arm.loop();
            drivetrain.loop();
            plane.loop();

            telemetry.update();
        }
    }
}