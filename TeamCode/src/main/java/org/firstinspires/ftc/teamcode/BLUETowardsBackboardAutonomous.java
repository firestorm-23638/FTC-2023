package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Point;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@Autonomous
public class BLUETowardsBackboardAutonomous extends LinearOpMode {
    OpenCvWebcam camera;
    FindProp propLoc = new FindProp();

    public void waitSeconds(double s) {
        double t = time;
        while (true) {
            if (t + s <= time) {
                return;
            }
        }
    }

    public void runOpMode() {
        ArmSubsystem arm = new ArmSubsystem(this.hardwareMap, telemetry, this.gamepad2, true);
        int monitorId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), monitorId);
        camera.setPipeline(propLoc);
        camera.setMillisecondsPermissionTimeout(5000);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
        int circlePos = -1;

        boolean hasCircle = false;

        SampleMecanumDrive sampleDrive = new SampleMecanumDrive(hardwareMap);

        sampleDrive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(180)));

        TrajectorySequence placeLeft = sampleDrive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(180)))
                .setReversed(true)
                .splineTo(new Vector2d(25, 8.5),0)
                .lineToLinearHeading(new Pose2d(15, 8.5, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(18.5, 35, Math.toRadians(270)))
                .build();

        TrajectorySequence placeMiddle = sampleDrive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(30, 0, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(21, 0, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(22, 34, Math.toRadians(270)))
                .build();

        TrajectorySequence placeRight = sampleDrive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(6, 0, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(26.5, -6.5, Math.toRadians(120)))
                .forward(20)
                .lineToLinearHeading(new Pose2d(31, 35, Math.toRadians(270)))
                .build();

        TrajectorySequence moveAwayLeft = sampleDrive.trajectorySequenceBuilder((new Pose2d(18.5, 32.5, Math.toRadians(270))))
                .forward(3)
                .strafeRight(24)
                .back(6)
                .build();

        TrajectorySequence moveAwayMiddle = sampleDrive.trajectorySequenceBuilder((new Pose2d(22, 32.5, Math.toRadians(270))))
                .forward(3)
                .strafeRight(32)
                .back(6)
                .build();

        TrajectorySequence moveAwayRight = sampleDrive.trajectorySequenceBuilder(new Pose2d(31, 34.25, Math.toRadians(270)))
                .forward(3)
                .strafeRight(38)
                .back(6)
                .build();

        TrajectorySequence forwardLeft = sampleDrive.trajectorySequenceBuilder(placeLeft.end())
                .forward(3)
                .build();

        TrajectorySequence forwardMiddle = sampleDrive.trajectorySequenceBuilder(placeMiddle.end())
                .forward(3)
                .build();

        TrajectorySequence forwardRight = sampleDrive.trajectorySequenceBuilder(placeRight.end())
                .forward(3)
                .build();

        waitForStart();
        arm.init();

        while (opModeIsActive()) {
            arm.setPivot(0.17);
            arm.closeLeftClaw();
            arm.closeRightClaw();

            if (hasCircle) {
                if (circlePos == 1) {
                    sampleDrive.followTrajectorySequence(placeLeft);
                    arm.setArmPos(910, true);
                    waitSeconds(0.5);
                    arm.openLeftClaw();
                    arm.openRightClaw();
                    sampleDrive.followTrajectorySequence(forwardLeft);
                    arm.setArmPos(10, true);
                    return;
                }
                else if (circlePos == 2) {
                    sampleDrive.followTrajectorySequence(placeMiddle);
                    arm.setArmPos(910, true);
                    waitSeconds(0.5);
                    arm.openLeftClaw();
                    arm.openRightClaw();
                    sampleDrive.followTrajectorySequence(forwardMiddle);
                    arm.setArmPos(10, true);
                    return;
                }
                else if (circlePos == 3) {
                    sampleDrive.followTrajectorySequence(placeRight);
                    arm.setArmPos(910, true);
                    waitSeconds(0.5);
                    arm.openLeftClaw();
                    arm.openRightClaw();
                    sampleDrive.followTrajectorySequence(forwardRight);
                    arm.setArmPos(10, true);
                    return;
                }


            }
            else if (propLoc.circleNum > 0) {
                if (propLoc.recentCircle.x < 350) {
                    circlePos = 1;
                    hasCircle = true;
                }
                else if (propLoc.recentCircle.x > 850) {
                    circlePos = 3;
                    hasCircle = true;
                }
                else {
                    circlePos = 2;
                    hasCircle = true;
                }


            }
            telemetry.addData("Circle pos", circlePos);
            telemetry.addData("Circle x", propLoc.recentCircle.x);
            telemetry.update();
        }
    }
}
