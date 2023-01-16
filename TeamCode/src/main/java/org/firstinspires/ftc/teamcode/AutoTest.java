package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm1;
import org.firstinspires.ftc.teamcode.subsystems.Arm2;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.vision.NormalizationDemoPipelineCone;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Disabled
@Autonomous(name= "auto", group= "Autonomous")
public class AutoTest extends LinearOpMode {

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    NormalizationDemoPipelineCone pipeline;

    ElapsedTime timeyBoi = new ElapsedTime();

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
        Arm2.initArm2(hardwareMap);

        pipeline = new NormalizationDemoPipelineCone(telemetry);

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
                .lineToLinearHeading(new Pose2d(startPose.getX() + -60.75, startPose.getY() + 2, Math.toRadians(70.25)))
                .addDisplacementMarker(55, () -> {
                    Arm1.forearmDown();
                })
                .build();

        TrajectorySequence sumo = drive.trajectorySequenceBuilder(trajSeq.end())
                .lineToLinearHeading(new Pose2d(startPose.getX() + -60.76, startPose.getY() + 2, Math.toRadians(70.25)))
                .build();

        TrajectorySequence left = drive.trajectorySequenceBuilder(sumo.end())
                .lineToLinearHeading(new Pose2d(startPose.getX() + -50, startPose.getY() + 1, Math.toRadians(90)))
                .addTemporalMarker(() -> {
                    Arm1.forearmUp();
                })
                .lineToLinearHeading(new Pose2d(startPose.getX() + -50, startPose.getY() -22, Math.toRadians(90)))
                .build();

        TrajectorySequence middle = drive.trajectorySequenceBuilder(sumo.end())
                .addTemporalMarker(() -> {
                    Arm1.forearmUp();
                })
                .lineToLinearHeading(new Pose2d(startPose.getX() + -50, startPose.getY() + 1, Math.toRadians(90)))
                .build();

        TrajectorySequence right = drive.trajectorySequenceBuilder(sumo.end())
                .lineToLinearHeading(new Pose2d(startPose.getX() + -50, startPose.getY() + 1, Math.toRadians(90)))
                .addTemporalMarker(() -> {
                    Arm1.forearmUp();
                })
                .lineToLinearHeading(new Pose2d(startPose.getX() + -50, startPose.getY() + 26, Math.toRadians(90)))                .build();

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

        boolean arm1Out = false;
        boolean arm1OutFlag = true;
        boolean senseCone = false;
        boolean arm1IsOut = false;
        boolean haveCone = false;
        double coneTimer = 420;
        boolean arm1In = false;
        boolean arm1IsIn = false;
        boolean arm1InTimerFlag = false;
        double arm1InTimer = 420;
        boolean arm2Up = true;
        boolean arm2UpSlow = false;
        boolean arm2IsUp = false;
        double arm2Timer = 420;
        boolean arm2Down = false;
        boolean arm2IsDown = false;
        boolean grab = false;
        boolean grabFlag = false;
        boolean cycleFlag = false;
        boolean lastCone = false;

        double cycle = 0;

        timeyBoi.reset();

        while(cycle < 7){
            if(arm1OutFlag){
                arm1OutFlag = false;
                if(cycle == 0){
                    Arm1.forearmSpecDown(.85);
                }
                else{
                    Arm1.forearmSpecDown(.85 + (0.025 * (cycle - 1)));
                }
                Arm1.openClaw();
                arm1OutFlag = false;
                arm1Out = true;
            }

            if(arm1Out){
                Arm1.moveArm(.5);
                if(Arm1.getArmPosition() > 400){
                    arm1Out = false;
                    senseCone = true;
                    Arm1.moveArm(0);
                }
            }

            if(senseCone){
                Arm1.moveArm(.15);
                if(Arm1.clawSensor.getDistance(DistanceUnit.CM) < 3){
                    senseCone = false;
                    arm1IsOut = true;
                    Arm1.moveArm(0);
                }
            }

            if(arm1IsOut && grab){
                Arm1.closeClaw();
                arm1IsOut = false;
                haveCone = true;
                grab = false;
            }

            if(haveCone){
                coneTimer = timeyBoi.time();
                haveCone = false;
            }

            if(timeyBoi.time() - coneTimer > .3 && !arm1In){
                arm1In = true;
                Arm1.forearmUpNoRotate();
            }

            if(timeyBoi.time() - coneTimer > .35){
                Arm1.rotaterUp();
                coneTimer = 420;
            }

            if(arm1In){
                Arm1.moveArm(-.7);
                if(Arm1.getArmPosition() < 5){
                    arm1In = false;
                    arm1IsIn = true;
                    Arm1.moveArm(0);
                    Arm1.resetArm();
                }
            }

            if(arm1IsIn){
                arm1IsIn = false;
                arm1InTimerFlag = true;
            }

            if(arm1InTimerFlag){
                arm1InTimer = timeyBoi.time();
                arm1InTimerFlag = false;
            }

            if(timeyBoi.time() - arm1InTimer > .45){
                Arm1.openClaw();
            }

            if(timeyBoi.time() - arm1InTimer > .55){
                arm1InTimerFlag = false;
                arm1InTimer = 420;
                arm2Up = true;
                cycleFlag = true;
            }

            if(arm2Up){
                if(cycle <= 5 && cycle != 0){
                    arm1OutFlag = true;
                }
                if(cycle != 0 && cycleFlag){
                    drive.followTrajectorySequence(sumo);
                    cycle++;
                    cycleFlag = false;
                }
                if(cycle == 6){
                    arm1OutFlag = false;
                }
                Arm2.moveArm(.85);
                if(Arm2.getArmPosition() > 300){
                    arm2UpSlow = true;
                    arm2Up = false;
                    Arm2.moveArm(0);
                }
            }

            if(arm2UpSlow){
                Arm2.moveArm(.65);
                if(Arm2.getArmPosition() > 480){
                    arm2IsUp = true;
                    arm2UpSlow = false;
                    Arm2.moveArm(0);
                }
            }

            if(arm2IsUp){
                arm2Timer = timeyBoi.time();
                arm2IsUp = false;
            }

            if(timeyBoi.time() - arm2Timer > .15 && !arm2Down) {
                if(cycle == 6){
                    Arm1.forearmDown();
                }
                arm2Timer = 420;
                arm2IsUp = false;
                arm2Down = true;
                grabFlag = true;
            }

            if(arm2Down){
                Arm2.moveArm(-1);
                if(Arm2.getArmPosition() < 300 && grabFlag){
                    grab = true;
                    if(cycle == 0){
                        cycle++;
                    }
                    grabFlag = false;

                }
                if(Arm2.getArmPosition() < 5){
                    arm2Down = false;
                    arm2IsDown = true;
                    Arm2.moveArm(0);
                    Arm2.resetArm();
                    if(cycle == 6 ){
                        break;
                    }
                }
            }
        }

        telemetry.addData("Hi ",  " dumbass");
        telemetry.update();

        if(tagOfInterest == null || tagOfInterest.id == LEFT){
            drive.followTrajectorySequence(left);
        }else if(tagOfInterest.id == MIDDLE){
            drive.followTrajectorySequence(middle);
        }else{
            drive.followTrajectorySequence(right);
        }

        //camera.stopStreaming();

        //camera.setPipeline(pipeline);

        /*camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 176, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Init Error", errorCode);
                telemetry.update();

            }
        });*/
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
    }
}