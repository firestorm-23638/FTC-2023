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
public class BLUEAwayBackboardAutonomous extends LinearOpMode {
    OpenCvWebcam camera;
    FindProp propLoc = new FindProp();

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

        Trajectory goRight = sampleDrive.trajectoryBuilder(new Pose2d(0, 0, Math.toRadians(180)), true)
                .splineTo(new Vector2d(25, -8.5),0)
                .build();

        Trajectory inchToLeft = sampleDrive.trajectoryBuilder(new Pose2d(0, 0, Math.toRadians(180)), true)
                .lineToLinearHeading(new Pose2d(6, 0, Math.toRadians(180)))
                .build();

        Trajectory goLeft = sampleDrive.trajectoryBuilder(new Pose2d(6, 0, Math.toRadians(180)), true)
                .lineToLinearHeading(new Pose2d(26, 6, Math.toRadians(240)))
                .build();

        Trajectory backupToBackdrop = sampleDrive.trajectoryBuilder(new Pose2d(25, 10, Math.toRadians(180)), true)
                .lineToLinearHeading(new Pose2d(10, 10, Math.toRadians(180)))
                .build();

        Trajectory prepareRightFromLeft = sampleDrive.trajectoryBuilder(new Pose2d(26, 6, Math.toRadians(240)), true)
                .lineToLinearHeading(new Pose2d(20, -20, Math.toRadians(180)))
                .build();

        Trajectory prepareRightFromMiddle = sampleDrive.trajectoryBuilder(new Pose2d(27, 0, Math.toRadians(180)), true)
                .lineToLinearHeading(new Pose2d(20, -20, Math.toRadians(180)))
                .build();

        Trajectory prepareRightFromRight = sampleDrive.trajectoryBuilder(new Pose2d(25, -8.5, Math.toRadians(180)), true)
                .lineToLinearHeading(new Pose2d(20, 0, Math.toRadians(180)))
                .build();

        Trajectory lineUp = sampleDrive.trajectoryBuilder(new Pose2d(20, -20, Math.toRadians(180)), true)
                .lineToLinearHeading(new Pose2d(55, -20, Math.toRadians(270)))
                .build();

        Trajectory lineUpLeft = sampleDrive.trajectoryBuilder(new Pose2d(20, -20, Math.toRadians(180)), true)
                .lineToLinearHeading(new Pose2d(57, -20, Math.toRadians(270)))
                .build();

        Trajectory lineUpRight = sampleDrive.trajectoryBuilder(new Pose2d(20, 0, Math.toRadians(180)), true)
                .lineToLinearHeading(new Pose2d(56, 0, Math.toRadians(270)))
                .build();

        Trajectory goToBackdropSide = sampleDrive.trajectoryBuilder(new Pose2d(55, -20, Math.toRadians(270)), true)
                .lineToLinearHeading(new Pose2d(55, 65, Math.toRadians(270)))
                .build();

        Trajectory goToBackdropSideLeft = sampleDrive.trajectoryBuilder(new Pose2d(57, -20, Math.toRadians(270)), true)
                .lineToLinearHeading(new Pose2d(57, 65, Math.toRadians(270)))
                .build();

        Trajectory backdropLeft = sampleDrive.trajectoryBuilder(new Pose2d(55, 65, Math.toRadians(270)), true)
                .lineToLinearHeading(new Pose2d(16.5, 81.5, Math.toRadians(270)))
                .build();

        Trajectory backdropMiddle = sampleDrive.trajectoryBuilder(new Pose2d(55, 65, Math.toRadians(270)), true)
                .lineToLinearHeading(new Pose2d(22, 80, Math.toRadians(270)))
                .build();

        Trajectory backdropRight = sampleDrive.trajectoryBuilder(new Pose2d(52, 65, Math.toRadians(270)), true)
                .lineToLinearHeading(new Pose2d(26, 80, Math.toRadians(270)))
                .build();

        Trajectory inchUpMiddle = sampleDrive.trajectoryBuilder(new Pose2d(22, 80, Math.toRadians(270)), true)
                .forward(2)
                .build();

        Trajectory inchUpLeft = sampleDrive.trajectoryBuilder(new Pose2d(14, 81, Math.toRadians(270)), true)
                .forward(2)
                .build();

        TrajectorySequence placeLeft = sampleDrive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(6, 0, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(26, 6, Math.toRadians(240)))
                .lineToLinearHeading(new Pose2d(52, -10, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(52, 65, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(16.5, 81.5, Math.toRadians(270)))
                .build();

        TrajectorySequence placeMiddle = sampleDrive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(27, 0, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(20, -20, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(52, -20, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(52, 65, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(22, 80, Math.toRadians(270)))
                .build();

        TrajectorySequence placeRight = sampleDrive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(180)))
                .splineTo(new Vector2d(25, -8.5),0)
                .lineToLinearHeading(new Pose2d(20, 0, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(52, 0, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(52, 65, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(26, 80, Math.toRadians(270)))
                .build();



        waitForStart();
        arm.init();
        //drive.init();

        while (opModeIsActive()) {
            if (hasCircle) {
                /*if (circlePos == 1 && propLoc.circleNum == 0) {

                }
                else if (propLoc.circleNum > 0) {//!(activeCircle.x - 160 > -20 && activeCircle.x - 160 < 20)) {

                }
                else {
                    //drive.zero();
                    //drive.loop();
                    //sampleDrive.followTrajectory(backup);
                }*/
                //sampleDrive.followTrajectory(backup);
                //return;
                /*if (circlePos == 1) {
                    sampleDrive.followTrajectory(goLeft);
                    return;
                }
                else if (circlePos == 2) {
                    sampleDrive.followTrajectory(backup);
                    return;
                }
                else if (circlePos == 3) {
                    sampleDrive.followTrajectory(goRight);
                    return;
                }*/
                if (circlePos == 3) {
                    /*sampleDrive.followTrajectory(goLeft);
                    sampleDrive.followTrajectory(backupToBackdrop);
                    sampleDrive.followTrajectory(backdropLeft);
                    arm.setArmPos(1000, true);
                    arm.setClaw(0.3);
                    arm.setArmPos(10, true);
                    return;*/
                    /*sampleDrive.followTrajectory(goRight);
                    sampleDrive.followTrajectory(prepareRightFromRight);
                    sampleDrive.followTrajectory(lineUp);
                    sampleDrive.followTrajectory(goToBackdropSide);
                    sampleDrive.followTrajectory(backdropRight);*/
                    sampleDrive.followTrajectorySequence(placeRight);
                    arm.setArmPos(1100, true);
                    arm.setLeftClaw(0);
                    arm.setArmPos(10, true);
                    return;
                }
                else if (circlePos == 2) {
                    /*sampleDrive.followTrajectory(goMiddle);
                    sampleDrive.followTrajectory(prepareRightFromMiddle);
                    sampleDrive.followTrajectory(lineUp);
                    sampleDrive.followTrajectory(goToBackdropSide);
                    sampleDrive.followTrajectory(backdropMiddle);*/
                    //sampleDrive.followTrajectory(backupToBackdropMiddle);
                    // sampleDrive.followTrajectory(backdropNearMiddle);
                    //sampleDrive.followTrajectory(backdropMiddle);
                    sampleDrive.followTrajectorySequence(placeMiddle);
                    arm.setArmPos(1100, true);
                    arm.setLeftClaw(0);
                    arm.setArmPos(10, true);
                    return;
                }
                else if (circlePos == 1) {
                    /*sampleDrive.followTrajectory(inchToLeft);
                    sampleDrive.followTrajectory(goLeft);
                    sampleDrive.followTrajectory(prepareRightFromLeft);
                    sampleDrive.followTrajectory(lineUpLeft);
                    sampleDrive.followTrajectory(goToBackdropSideLeft);
                    sampleDrive.followTrajectory(backdropLeft);*/
                    // sampleDrive.followTrajectory(backupToBackdrop);
                    //sampleDrive.followTrajectory(backdropRight);
                    //arm.setArmPos(1000, true);
                    //arm.setClaw(0.3);
                    //arm.setArmPos(10, true);
                    sampleDrive.followTrajectorySequence(placeLeft);
                    arm.setArmPos(1100, true);
                    arm.setLeftClaw(0);
                    arm.setArmPos(10, true);
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
