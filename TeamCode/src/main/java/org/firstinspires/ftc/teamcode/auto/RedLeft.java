package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm1;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
@Disabled

@Autonomous(name= "RedLeft", group= "Autonomous")
public class RedLeft extends LinearOpMode {

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Arm1.initArm1(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "camera"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        Pose2d startPose = new Pose2d(-63.5, -36);

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)

                /*
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    Arm.armSpecDown(90);
                })
                .lineTo(new Vector2d(startPose.getX() + 50, startPose.getY() - 28.5))
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    Arm.closeClaw();
                })
                .waitSeconds(.5)
                .addTemporalMarker(() -> {
                    Arm.armTop();
                })
                .UNSTABLE_addDisplacementMarkerOffset(10, () -> {
                    Arm.swinger.setPosition(.23);
                })
                .lineTo(new Vector2d(startPose.getX() + 50, startPose.getY() + 9))
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    Arm.armTopKinda();
                })
                .waitSeconds(.5)
                .addTemporalMarker(() -> {
                    Arm.openClaw();
                })
                .waitSeconds(.5)
                .addTemporalMarker(() -> {
                    Arm.swinger.setPosition(.5);
                })



                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    Arm.armSpecDown(60);
                })
                .lineTo(new Vector2d(startPose.getX() + 50, startPose.getY() - 28.5))
                .addTemporalMarker(() -> {
                    Arm.closeClaw();
                })
                .waitSeconds(.5)
                .addTemporalMarker(() -> {
                    Arm.armTop();
                })
                .UNSTABLE_addDisplacementMarkerOffset(10, () -> {
                    Arm.swinger.setPosition(.23);
                })
                .lineTo(new Vector2d(startPose.getX() + 50, startPose.getY() + 9))
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    Arm.armTopKinda();
                })
                .waitSeconds(.5)
                .addTemporalMarker(() -> {
                    Arm.openClaw();
                })
                .waitSeconds(.5)
                .addTemporalMarker(() -> {
                    Arm.swinger.setPosition(.5);
                })



                .waitSeconds(.5)
                .addTemporalMarker(() -> {
                    Arm.armSpecDown(0);
                })
                .lineTo(new Vector2d(startPose.getX() + 50, startPose.getY() - 28.5))
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    Arm.closeClaw();
                })
                .waitSeconds(.5)
                .addTemporalMarker(() -> {
                    Arm.armTop();
                })
                .UNSTABLE_addDisplacementMarkerOffset(10, () -> {
                    Arm.swinger.setPosition(.23);
                })
                .lineTo(new Vector2d(startPose.getX() + 50, startPose.getY() + 9))
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    Arm.armTopKinda();
                })
                .waitSeconds(.5)
                .addTemporalMarker(() -> {
                    Arm.openClaw();
                })
                .waitSeconds(.5)
                .addDisplacementMarker(() -> {
                    Arm.swinger.setPosition(.5);
                })
                */
                .build();


        TrajectorySequence left = drive.trajectorySequenceBuilder(trajSeq.end())
                .forward(32)
                .build();

        TrajectorySequence middle = drive.trajectorySequenceBuilder(trajSeq.end())
                .forward(8)
                .build();

        TrajectorySequence right = drive.trajectorySequenceBuilder(trajSeq.end())
                .back(17)
                .build();

        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectorySequence(trajSeq);

        Arm1.armSpecDown(0);


        if(tagOfInterest == null || tagOfInterest.id == LEFT){
            drive.followTrajectorySequence(left);
        }else if(tagOfInterest.id == MIDDLE){
            drive.followTrajectorySequence(middle);
        }else{
            drive.followTrajectorySequence(right);
        }
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
    }
}