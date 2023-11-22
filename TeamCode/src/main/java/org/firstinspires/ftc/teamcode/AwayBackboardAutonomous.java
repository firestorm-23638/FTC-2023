package org.firstinspires.ftc.teamcode;

import org.opencv.core.Point;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

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
public class AwayBackboardAutonomous extends LinearOpMode {
    OpenCvWebcam camera;
    FindProp propLoc = new FindProp();
    public void runOpMode() {
        int monitorId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), monitorId);
        camera.setPipeline(propLoc);
        camera.setMillisecondsPermissionTimeout(5000);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
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
        DrivetrainSubsystem drive = new DrivetrainSubsystem(hardwareMap, telemetry);

        sampleDrive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(180)));
        Trajectory backup = sampleDrive.trajectoryBuilder(new Pose2d(0, 0, Math.toRadians(180)), true)
                .lineToLinearHeading(new Pose2d(29.5, 0, Math.toRadians(180)))
                .build();
        Trajectory goLeft = sampleDrive.trajectoryBuilder(new Pose2d(0, 0, Math.toRadians(180)), true)
                .strafeTo(new Vector2d(20, 12.5))
                .build();

        Trajectory goRight = sampleDrive.trajectoryBuilder(new Pose2d(0, 0, Math.toRadians(180)), true)
                .splineTo(new Vector2d(30, -11), Math.toRadians(270))
                .build();

        waitForStart();
        //drive.init();

        while (opModeIsActive()) {
            circleTick ++;
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
                sampleDrive.followTrajectory(backup);
                return;
                /*if (circlePos == 1) {
                    sampleDrive.followTrajectory(backup);
                    return;
                }
                else if (circlePos == 2) {
                    sampleDrive.followTrajectory(backup);
                    return;
                }
                else if (circlePos == 3) {
                    sampleDrive.followTrajectory(goLeft);
                    return;
                }*/
            }
            else if (circleTick > 100000 && circlePos == -1) {
                circlePos = 1;
                hasCircle = true;
            }
            else if (propLoc.circleNum > 0) {
                if (propLoc.recentCircle.x < 150) {
                    circlePos = 3;
                    activeCircle = propLoc.recentCircle;
                    hasCircle = true;
                }
                else if (propLoc.recentCircle.x > 0){
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
