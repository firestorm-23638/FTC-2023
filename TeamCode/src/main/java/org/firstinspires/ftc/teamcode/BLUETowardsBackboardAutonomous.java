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

        long circleTick = 0;
        int circlePos = -1;

        Point activeCircle = new Point(0, 0);

        boolean hasCircle = false;

        SampleMecanumDrive sampleDrive = new SampleMecanumDrive(hardwareMap);

        sampleDrive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(180)));

        Trajectory goMiddle = sampleDrive.trajectoryBuilder(new Pose2d(0, 0, Math.toRadians(180)), true)
                .lineToLinearHeading(new Pose2d(27, 0, Math.toRadians(180)))
                .build();

        Trajectory goLeft = sampleDrive.trajectoryBuilder(new Pose2d(0, 0, Math.toRadians(180)), true)
                .splineTo(new Vector2d(25, 8.5),0)
                .build();

        Trajectory inchToRight = sampleDrive.trajectoryBuilder(new Pose2d(0, 0, Math.toRadians(180)), true)
                .lineToLinearHeading(new Pose2d(6, 0, Math.toRadians(180)))
                .build();

        Trajectory goRight = sampleDrive.trajectoryBuilder(new Pose2d(6, 0, Math.toRadians(180)), true)
                .lineToLinearHeading(new Pose2d(26, -6, Math.toRadians(120)))
                .build();

        Trajectory backupToBackdrop = sampleDrive.trajectoryBuilder(new Pose2d(25, 10, Math.toRadians(180)), true)
                .lineToLinearHeading(new Pose2d(22, 10, Math.toRadians(180)))
                .build();

        Trajectory backupToBackdropMiddle = sampleDrive.trajectoryBuilder(new Pose2d(27, 0, Math.toRadians(180)), true)
                .lineToLinearHeading(new Pose2d(24, 0, Math.toRadians(180)))
                .build();

        Trajectory backupToBackdropRight = sampleDrive.trajectoryBuilder(new Pose2d(26, -6, Math.toRadians(120)), true)
                .lineToLinearHeading(new Pose2d(10, 0, Math.toRadians(120)))
                .build();

        Trajectory backdropLeft = sampleDrive.trajectoryBuilder(new Pose2d(22, 10, Math.toRadians(180)), true)
                .lineToLinearHeading(new Pose2d(16, 33.5, Math.toRadians(270)))
                .build();

        Trajectory backdropMiddle = sampleDrive.trajectoryBuilder(new Pose2d(24, 0, Math.toRadians(180)), true)
                .lineToLinearHeading(new Pose2d(21, 33.5, Math.toRadians(270)))
                .build();

        Trajectory backdropRight = sampleDrive.trajectoryBuilder(new Pose2d(26, -6, Math.toRadians(120)), true)
                .lineToLinearHeading(new Pose2d(29, 34, Math.toRadians(270)))
                .build();

        Trajectory getOutOfTheWayLeft = sampleDrive.trajectoryBuilder(new Pose2d(16, 30.5, Math.toRadians(270)), true)
                .strafeRight(26)
                .build();

        Trajectory getOutOfTheWayMiddle = sampleDrive.trajectoryBuilder(new Pose2d(23, 30.5, Math.toRadians(270)), true)
                .strafeRight(34)
                .build();

        Trajectory getOutOfTheWayRight = sampleDrive.trajectoryBuilder(new Pose2d(29, 31, Math.toRadians(270)), true)
                .strafeRight(40)
                .build();

        Trajectory shiftUpLeft = sampleDrive.trajectoryBuilder(new Pose2d(16, 33.5, Math.toRadians(270)), true)
                .forward(3)
                .build();

        Trajectory shiftUpMiddle = sampleDrive.trajectoryBuilder(new Pose2d(23, 33.5, Math.toRadians(270)), true)
                .forward(3)
                .build();

        Trajectory shiftUpRight = sampleDrive.trajectoryBuilder(new Pose2d(29, 34, Math.toRadians(270)), true)
                .forward(3)
                .build();


        TrajectorySequence placeLeft = sampleDrive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(180)))
                .setReversed(true)
                .splineTo(new Vector2d(25, 8.5),0)
                .lineToLinearHeading(new Pose2d(22, 8.5, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(16, 34, Math.toRadians(270)))
                .build();

        TrajectorySequence placeMiddle = sampleDrive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(27, 0, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(21, 0, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(19, 35, Math.toRadians(270)))
                .build();

        TrajectorySequence placeRight = sampleDrive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(6, 0, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(26, -6, Math.toRadians(120)))
                .lineToLinearHeading(new Pose2d(14, 0, Math.toRadians(120)))
                .lineToLinearHeading(new Pose2d(29, 35, Math.toRadians(270)))
                .build();

        TrajectorySequence moveAwayLeft = sampleDrive.trajectorySequenceBuilder((new Pose2d(16, 32.5, Math.toRadians(270))))
                .forward(3)
                .strafeRight(24)
                .back(3)
                .build();

        TrajectorySequence moveAwayMiddle = sampleDrive.trajectorySequenceBuilder((new Pose2d(21, 32.5, Math.toRadians(270))))
                .forward(3)
                .strafeRight(28)
                .back(3)
                .build();

        TrajectorySequence moveAwayRight = sampleDrive.trajectorySequenceBuilder((new Pose2d(29, 32.5, Math.toRadians(270))))
                .forward(3)
                .strafeRight(36)
                .back(3)
                .build();

        waitForStart();
        arm.init();
        //drive.init();

        while (opModeIsActive()) {
            telemetry.addData("circle rad", propLoc.circleRadius);
            telemetry.addData("Time", time);
            arm.setLeftClaw(0.18);
            arm.setPivot(0.2);
            if (hasCircle) {
                if (circlePos == 1) {
                    sampleDrive.followTrajectorySequence(placeLeft);
                    /*sampleDrive.followTrajectory(goLeft);
                    sampleDrive.followTrajectory(backupToBackdrop);
                    sampleDrive.followTrajectory(backdropLeft);*/
                    arm.setArmPos(530, true);
                    waitSeconds(0.5);
                    arm.setLeftClaw(0);
                    //arm.setRawArm(-0.25);
                    sampleDrive.followTrajectorySequence(moveAwayLeft);
                    /*sampleDrive.followTrajectory(shiftUpLeft);
                    sampleDrive.followTrajectory(getOutOfTheWayLeft);*/
                    arm.setArmPos(10, true);
                    return;
                }
                else if (circlePos == 2) {
                    sampleDrive.followTrajectorySequence(placeMiddle);
                    /*sampleDrive.followTrajectory(goMiddle);
                    sampleDrive.followTrajectory(backupToBackdropMiddle);
                   // sampleDrive.followTrajectory(backdropNearMiddle);
                    sampleDrive.followTrajectory(backdropMiddle);*/
                    arm.setArmPos(530, true);
                    waitSeconds(0.5);
                    arm.setLeftClaw(0);
                    //arm.setRawArm(-0.25);
                    sampleDrive.followTrajectorySequence(moveAwayMiddle);
                    /*sampleDrive.followTrajectory(shiftUpMiddle);
                    sampleDrive.followTrajectory(getOutOfTheWayMiddle);*/
                    arm.setArmPos(10, true);
                    return;
                }
                else if (circlePos == 3) {
                    sampleDrive.followTrajectorySequence(placeRight);
                    /*sampleDrive.followTrajectory(inchToRight);
                    sampleDrive.followTrajectory(goRight);
                    //sampleDrive.followTrajectory(backupToBackdropRight);
                    sampleDrive.followTrajectory(backdropRight);*/
                    arm.setArmPos(530, true);
                    waitSeconds(0.5);
                    arm.setLeftClaw(0);
                    //arm.setRawArm(-0.25);
                    sampleDrive.followTrajectorySequence(moveAwayRight);
                    /*sampleDrive.followTrajectory(shiftUpRight);
                    sampleDrive.followTrajectory(getOutOfTheWayRight);*/
                    arm.setArmPos(10, true);
                    //sampleDrive.followTrajectory(getOutOfTheWayRight);
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
            //drive.setToValues(this.gamepad1.left_stick_x, this.gamepad1.left_stick_y, this.gamepad1.right_stick_x, 0);
            //drive.loop();
        }
    }
}
