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
public class REDTowardsBackboardAutonomous extends LinearOpMode {
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

        long circleTick = 0;
        int circlePos = -1;

        Point activeCircle = new Point(0, 0);

        boolean hasCircle = false;

        SampleMecanumDrive sampleDrive = new SampleMecanumDrive(hardwareMap);

        sampleDrive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(180)));

        Trajectory goMiddle = sampleDrive.trajectoryBuilder(new Pose2d(0, 0, Math.toRadians(180)), true)
                .lineToLinearHeading(new Pose2d(28, 0, Math.toRadians(180)))
                .build();

        Trajectory goRight = sampleDrive.trajectoryBuilder(new Pose2d(0, 0, Math.toRadians(180)), true)
                .splineTo(new Vector2d(25, -8.5),0)
                .build();

        Trajectory inchToLeft = sampleDrive.trajectoryBuilder(new Pose2d(0, 0, Math.toRadians(180)), true)
                .lineToLinearHeading(new Pose2d(6, 0, Math.toRadians(180)))
                .build();

        Trajectory goLeft = sampleDrive.trajectoryBuilder(new Pose2d(6, 0, Math.toRadians(180)), true)
                .lineToLinearHeading(new Pose2d(26, 6, Math.toRadians(240)))
                .build();

        Trajectory backupToBackdrop = sampleDrive.trajectoryBuilder(new Pose2d(25, -8.5, Math.toRadians(180)), true)
                .lineToLinearHeading(new Pose2d(18, -8.5, Math.toRadians(180)))
                .build();

        Trajectory backupToBackdropMiddle = sampleDrive.trajectoryBuilder(new Pose2d(28, 0, Math.toRadians(180)), true)
                .lineToLinearHeading(new Pose2d(10, 0, Math.toRadians(180)))
                .build();

        Trajectory backupToBackdropLeft = sampleDrive.trajectoryBuilder(new Pose2d(26, 5, Math.toRadians(240)), true)
                .lineToLinearHeading(new Pose2d(10, 0, Math.toRadians(240)))
                .build();

        Trajectory backdropLeft = sampleDrive.trajectoryBuilder(new Pose2d(26, 5, Math.toRadians(240)), true)
                .lineToLinearHeading(new Pose2d(33, -36.5, Math.toRadians(90)))
                .build();

        Trajectory backdropMiddle = sampleDrive.trajectoryBuilder(new Pose2d(10, 0, Math.toRadians(180)), true)
                .lineToLinearHeading(new Pose2d(27, -36.5, Math.toRadians(90)))
                .build();

        Trajectory backdropRight = sampleDrive.trajectoryBuilder(new Pose2d(18, -8.5, Math.toRadians(180)), true)
                .lineToLinearHeading(new Pose2d(18, -36.5, Math.toRadians(90)))
                .build();

        Trajectory getOutOfTheWayLeft = sampleDrive.trajectoryBuilder(new Pose2d(33, -36.5, Math.toRadians(90)), true)
                .strafeLeft(33.5)
                .build();

        Trajectory getOutOfTheWayMiddle = sampleDrive.trajectoryBuilder(new Pose2d(27, -36.5, Math.toRadians(90)), true)
                .strafeLeft(27.5)
                .build();

        Trajectory getOutOfTheWayRight = sampleDrive.trajectoryBuilder(new Pose2d(18, -36.5, Math.toRadians(90)), true)
                .strafeLeft(18.5)
                .build();

        TrajectorySequence placeLeft = sampleDrive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(6, 0, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(26, 6, Math.toRadians(240)))
                .lineToLinearHeading(new Pose2d(33, -31.75, Math.toRadians(90)))
                .build();

        TrajectorySequence placeMiddle = sampleDrive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(28, 0, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(20, 0, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(24, -31.75, Math.toRadians(90)))
                .build();

        TrajectorySequence placeRight = sampleDrive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(180)))
                .setReversed(true)
                .splineTo(new Vector2d(25, -8.5),0)
                .lineToLinearHeading(new Pose2d(18, -8.5, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(18, -31.75, Math.toRadians(90)))
                .build();

        TrajectorySequence moveAwayLeft = sampleDrive.trajectorySequenceBuilder(new Pose2d(33, -31.5, Math.toRadians(90)))
                .forward(3)
                .strafeLeft(37)
                .back(5)
                .build();

        TrajectorySequence moveAwayMiddle = sampleDrive.trajectorySequenceBuilder(new Pose2d(24, -31.5, Math.toRadians(90)))
                .forward(3)
                .strafeLeft(33)
                .back(5)
                .build();

        TrajectorySequence moveAwayRight = sampleDrive.trajectorySequenceBuilder(new Pose2d(18, -31.5, Math.toRadians(90)))
                .forward(3)
                .strafeLeft(23)
                .back(5)
                .build();

        waitForStart();
        arm.init();
        //drive.init();

        while (opModeIsActive()) {
            arm.closeLeftClaw();
            arm.closeRightClaw();

            if (hasCircle) {
                if (circlePos == 3) {
                    sampleDrive.followTrajectorySequence(placeRight);
                    arm.setArmPos(1000, true);
                    waitSeconds(0.5);
                    arm.openLeftClaw();
                    arm.openRightClaw();
                    sampleDrive.followTrajectorySequence(moveAwayRight);
                    arm.setArmPos(30, true);
                    return;
                }
                else if (circlePos == 2) {
                    sampleDrive.followTrajectorySequence(placeMiddle);
                    arm.setArmPos(1000, true);
                    waitSeconds(0.5);
                    arm.openLeftClaw();
                    arm.openRightClaw();
                    sampleDrive.followTrajectorySequence(moveAwayMiddle);
                    arm.setArmPos(30, true);
                    return;
                }
                else if (circlePos == 1) {
                    sampleDrive.followTrajectorySequence(placeLeft);
                    arm.setArmPos(1000, true);
                    waitSeconds(0.5);
                    arm.openLeftClaw();
                    arm.openRightClaw();
                    sampleDrive.followTrajectorySequence(moveAwayLeft);
                    arm.setArmPos(30, true);
                    return;
                }

            }
            else if (propLoc.circleNum > 0) {
                if (propLoc.recentCircle.x < 350) {
                    circlePos = 1;
                    activeCircle = propLoc.recentCircle;
                    hasCircle = true;
                }
                else if (propLoc.recentCircle.x > 850) {
                    circlePos = 3;
                    activeCircle = propLoc.recentCircle;
                    hasCircle = true;
                }
                else {
                    circlePos = 2;
                    activeCircle = propLoc.recentCircle;
                    hasCircle = true;
                }


            }
            telemetry.addData("Circle pos", circlePos);
            telemetry.addData("Circle x", propLoc.recentCircle.x);
            telemetry.update();
        }
    }
}
