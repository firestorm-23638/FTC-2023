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

// you should see this :
@Autonomous
public class REDAwayBackboardAutonomous extends LinearOpMode {
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

        Trajectory goLeft = sampleDrive.trajectoryBuilder(new Pose2d(0, 0, Math.toRadians(180)), true)
                .splineTo(new Vector2d(25, 8.5),0)
                .build();

        Trajectory inchToRight = sampleDrive.trajectoryBuilder(new Pose2d(0, 0, Math.toRadians(180)), true)
                .lineToLinearHeading(new Pose2d(8, 0, Math.toRadians(180)))
                .build();

        Trajectory goRight = sampleDrive.trajectoryBuilder(new Pose2d(8, 0, Math.toRadians(180)), true)
                .lineToLinearHeading(new Pose2d(27, -5, Math.toRadians(120)))
                .build();

        Trajectory backupToBackdrop = sampleDrive.trajectoryBuilder(new Pose2d(25, 10, Math.toRadians(180)), true)
                .lineToLinearHeading(new Pose2d(10, 10, Math.toRadians(180)))
                .build();

        Trajectory backupToBackdropMiddle = sampleDrive.trajectoryBuilder(new Pose2d(28, 0, Math.toRadians(180)), true)
                .lineToLinearHeading(new Pose2d(10, 0, Math.toRadians(180)))
                .build();

        Trajectory backdropLeft = sampleDrive.trajectoryBuilder(new Pose2d(10, 10, Math.toRadians(180)), true)
                .lineToLinearHeading(new Pose2d(16, 37, Math.toRadians(270)))
                .build();

        Trajectory backdropMiddle = sampleDrive.trajectoryBuilder(new Pose2d(10, 0, Math.toRadians(180)), true)
                .lineToLinearHeading(new Pose2d(18, 46, Math.toRadians(180)))
                .build();


        Trajectory backdropRight = sampleDrive.trajectoryBuilder(new Pose2d(10, 10, Math.toRadians(120)), true)
                .lineToLinearHeading(new Pose2d(20, 50, Math.toRadians(270)))
                .build();

        TrajectorySequence placeLeft = sampleDrive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(180)))
                .setReversed(true)
                .splineTo(new Vector2d(25, 8.5),0)
                /*.lineToLinearHeading(new Pose2d(20, 0, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(52, 0, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(52, -65, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(26, -80, Math.toRadians(270)))*/
                .build();

        TrajectorySequence placeMiddle = sampleDrive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(30, 0, Math.toRadians(180)))
                /*.lineToLinearHeading(new Pose2d(20, 20, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(52, 20, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(52, -65, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(22, -80, Math.toRadians(270)))*/
                .build();

        TrajectorySequence placeRight = sampleDrive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(6, 0, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(26, -6, Math.toRadians(120)))
                /*.lineToLinearHeading(new Pose2d(26, -6, Math.toRadians(240)))
                .lineToLinearHeading(new Pose2d(52, 5, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(52, -65, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(16.5, -80, Math.toRadians(270)))*/
                .build();

        TrajectorySequence comeForwardLeft = sampleDrive.trajectorySequenceBuilder(new Pose2d(16.5, 80, Math.toRadians(270)))
                .forward(3)
                .build();

        TrajectorySequence comeForwardMiddle = sampleDrive.trajectorySequenceBuilder(new Pose2d(22, 80, Math.toRadians(270)))
                .forward(3)
                .build();

        TrajectorySequence comeForwardRight = sampleDrive.trajectorySequenceBuilder(new Pose2d(26, 80, Math.toRadians(270)))
                .forward(3)
                .build();

        waitForStart();
        arm.init();
        //drive.init();

        while (opModeIsActive()) {
            arm.closeLeftClaw();
            arm.closeRightClaw();

            if (hasCircle) {
                if (circlePos == 1) {
                    sampleDrive.followTrajectorySequence(placeLeft);
                    /*arm.setArmPos(530, true);
                    waitSeconds(0.5);
                    arm.openLeftClaw();
                    arm.openRightClaw();
                    waitSeconds(0.5);
                    arm.setArmPos(10, true);*/
                    return;
                }
                else if (circlePos == 2) {
                    sampleDrive.followTrajectorySequence(placeMiddle);
                    /*arm.setArmPos(530, true);
                    waitSeconds(0.5);
                    arm.openLeftClaw();
                    arm.openRightClaw();
                    waitSeconds(0.5);
                    arm.setArmPos(10, true);*/
                    return;
                }
                else if (circlePos == 3) {
                    sampleDrive.followTrajectorySequence(placeRight);
                    /*arm.setArmPos(530, true);
                    waitSeconds(0.5);
                    arm.openLeftClaw();
                    arm.openRightClaw();
                    waitSeconds(0.5);
                    arm.setArmPos(10, true);*/
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
