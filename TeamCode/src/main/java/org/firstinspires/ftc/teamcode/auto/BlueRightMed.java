package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm1;
import org.firstinspires.ftc.teamcode.subsystems.Arm2;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.vision.NormalizationDemoPipelineConeBlue;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name= "BlueRightMed", group= "Autonomous")
public class BlueRightMed extends LinearOpMode {

    OpenCvCamera camera;
    OpenCvCamera camera2;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    NormalizationDemoPipelineConeBlue pipeline;

    ElapsedTime timeyBoi = new ElapsedTime();

    int YLower = 30;
    int CrLower = 80;
    int CbLower = 145;
    int YUpper = 160;
    int CrUpper = 130;
    int CbUpper = 195;

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
        DriveTrain.initDriveTrain(hardwareMap);

        Arm1.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE);

        pipeline = new NormalizationDemoPipelineConeBlue(telemetry);

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
                .addTemporalMarker(() -> {
                    camera.closeCameraDevice();

                    camera2 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "camera2"), cameraMonitorViewId);
                    pipeline = new NormalizationDemoPipelineConeBlue(telemetry);
                    camera2.setPipeline(pipeline);

                    camera2.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                        @Override
                        public void onOpened() {
                            camera2.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
                        }

                        @Override
                        public void onError(int errorCode) {
                            telemetry.addData("Camera Init Error", errorCode);
                            telemetry.update();

                        }
                    });

                    pipeline.setYMin(YLower);
                    pipeline.setYMax(YUpper);
                    pipeline.setCrMin(CrLower);
                    pipeline.setCrMax(CrUpper);
                    pipeline.setCbMin(CbLower);
                    pipeline.setCbMax(CbUpper);
                })
                .setVelConstraint(new TrajectoryVelocityConstraint() {
                    @Override
                    public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                        return 75;
                    }
                })
                .setAccelConstraint(new TrajectoryAccelerationConstraint() {
                    @Override
                    public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                        return 40;
                    }
                })
                .lineTo(new Vector2d(startPose.getX() + -42.75, startPose.getY() - 0))
                .turn(Math.toRadians(102.5))
                .lineTo(new Vector2d(startPose.getX() + -42.75, startPose.getY() - 3))
                .resetVelConstraint()
                .resetAccelConstraint()
                .build();

        TrajectorySequence sumo = drive.trajectorySequenceBuilder(trajSeq.end())
                .lineTo(new Vector2d(startPose.getX() + -42.751, startPose.getY() - 3))
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

        Arm2.closeServo();

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
        boolean rotate = false;
        double senseTimer = 420;

        int lastX = 0;
        int currentX = 0;
        int target = 940;
        double power = 0.3;
        boolean move = false;
        boolean moving = false;
        int count = 0;
        boolean search = true;
        int camCycle = 0;
        double targetEnc = 0;
        double currentEnc = 0;
        double lastEnc = 0;
        double searchTime = 0;
        boolean smoll = false;
        boolean broken = false;
        double totalCurrent = 0;
        boolean totalFlag = false;
        double totalTimer = 0;
        double arm2Current = 0;
        boolean arm2CurrentFlag = false;
        double arm2CurrentTimer = 0;

        double cycle = 0;

        timeyBoi.reset();

        TrajectorySequence correct = drive.trajectorySequenceBuilder(trajSeq.end())
                .turn(-(Math.toRadians(((pipeline.getXContour() - 159) / 3))))
                .build();

        currentX = pipeline.getXContour();

        while(cycle < 7){
            camCycle++;
            totalCurrent = (DriveTrain.leftFront.getCurrent(CurrentUnit.MILLIAMPS) + DriveTrain.leftBack.getCurrent(CurrentUnit.MILLIAMPS) + DriveTrain.rightFront.getCurrent(CurrentUnit.MILLIAMPS) + DriveTrain.rightBack.getCurrent(CurrentUnit.MILLIAMPS) + Arm1.arm1Left.getCurrent(CurrentUnit.MILLIAMPS) + Arm1.arm1Right.getCurrent(CurrentUnit.MILLIAMPS) + Arm2.arm2Left.getCurrent(CurrentUnit.MILLIAMPS) + Arm2.arm2Right.getCurrent(CurrentUnit.MILLIAMPS));
            arm2Current = (Arm2.arm2Left.getCurrent(CurrentUnit.MILLIAMPS) + Arm2.arm2Right.getCurrent(CurrentUnit.MILLIAMPS));
            if(totalCurrent > 32000 && !totalFlag){
                totalFlag = true;
                totalTimer = timeyBoi.time();
            }
            if(totalFlag){
                if(totalCurrent < 32000){
                    totalFlag = false;
                }
                else if(timeyBoi.time() - totalTimer > 1){
                    broken = true;
                    break;
                }
            }
            if(arm2Current > 18000 && !arm2CurrentFlag){
                arm2CurrentFlag = true;
                arm2CurrentTimer = timeyBoi.time();
            }
            if(arm2CurrentFlag){
                if(arm2Current < 18000){
                    arm2CurrentFlag = false;
                }
                else if(timeyBoi.time() - arm2CurrentTimer > 1){
                    broken = true;
                    break;
                }
            }
            if(search){
                currentX = pipeline.getXContour();
                if(!moving && !move){
                    if((currentX < target - 30 || currentX > target + 30) && timeyBoi.time() - searchTime > .5 && currentX != 0){
                        move = true;
                        moving = true;
                    }
                }
                if(moving){
                    currentEnc = DriveTrain.getMiddlePosition();
                    if(move){
                        if(currentX > target){
                            //Turn Right
                            targetEnc = currentEnc - (Math.abs(currentX - target) * 2.45);
                            move = false;
                        }
                        else if(currentX < target){
                            //Turn Left
                            targetEnc = currentEnc + (Math.abs(currentX - target) * 2.45);
                            move = false;
                        }
                        if(Math.abs(targetEnc - currentEnc) < 500){
                            power += .04;
                            smoll = true;
                        }
                    }
                    if(camCycle % 10 == 0){
                        if (Math.abs(lastEnc - currentEnc) <= 2 && move){
                            count++;
                        }
                        else{
                            count = 0;
                        }
                        if (count == 10) {
                            power += 0.01;
                            count = 0;
                        }
                    }
                    if(camCycle % 10 == 0){
                        if(Math.abs(lastEnc - currentEnc) > 10 && move && lastX != 0){
                            power -= 0.01;
                        }
                        lastEnc = currentEnc;
                    }
                    if(currentEnc > targetEnc - 30 && currentEnc < targetEnc + 30){
                        moving = false;
                        searchTime = timeyBoi.time();
                        DriveTrain.turn(0);
                        count = 0;
                        if(smoll){
                            power -= .04;
                            smoll = false;
                        }
                    }
                    else if(currentEnc < targetEnc){
                        DriveTrain.turn(-(power + ((Math.abs(targetEnc - currentEnc) / 3500))));
                    }
                    else if(currentEnc > targetEnc){
                        DriveTrain.turn((power + ((Math.abs(targetEnc - currentEnc) / 3500))));
                    }
                }
            }
            if(arm1OutFlag){
                if(cycle == 0){
                    Arm1.forearmSpecDown(.85);
                }
                else{
                    Arm1.forearmSpecDown(.85 + (0.025 * (cycle - 1)));
                    Arm1.rotaterUp();
                }
                Arm1.openClaw();
                arm1OutFlag = false;
                arm1Out = true;
                rotate = true;
            }

            if(arm1Out){
                Arm1.moveArm(.43);
                if(Arm1.getArmPosition() > 25 && rotate){
                    Arm1.rotaterDown();
                    rotate = false;
                }
                if(Arm1.getArmPosition() > 395 - (cycle * 5)){
                    arm1Out = false;
                    senseCone = true;
                    senseTimer = timeyBoi.time();
                    Arm1.moveArm(0);
                    search = true;
                }
            }

            if(senseCone && !move && !moving){
                Arm1.moveArm(.15);
                if(Arm1.clawSensor.getDistance(DistanceUnit.CM) < 5){
                    senseCone = false;
                    Arm1.moveArm(0);
                }
            }

            if(senseCone){
                if(Arm1.clawSensor.getDistance(DistanceUnit.CM) < 5){
                    senseCone = false;
                    Arm1.moveArm(0);
                }
            }

            if(grab){
                Arm1.moveArm(.15);
                if(Arm1.clawSensor.getDistance(DistanceUnit.CM) < 3){
                    senseCone = false;
                    arm1IsOut = true;
                    senseTimer = 420;
                    Arm1.moveArm(0);
                }
                if(timeyBoi.time() - senseTimer > 3){
                    Arm1.armDown();
                    Arm2.armDown();
                    Arm1.forearmUpNoRotate();
                    break;
                }
            }

            if(arm1IsOut && grab){
                Arm1.closeClaw();
                arm1IsOut = false;
                haveCone = true;
                grab = false;
                search = false;
                DriveTrain.turn(0);
            }

            if(haveCone){
                coneTimer = timeyBoi.time();
                haveCone = false;
            }

            if(timeyBoi.time() - coneTimer > .3 && !arm1In){
                arm1In = true;
                Arm1.forearmUpNoRotate();
            }

            if(timeyBoi.time() - coneTimer > .5){
                Arm1.rotaterUp();
                coneTimer = 420;
            }

            if(arm1In){
                if(cycle == 1){
                    Arm1.moveArm(-.5);
                }else{
                    Arm1.moveArm(-.5);
                }
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

            if(timeyBoi.time() - arm1InTimer > .3){
                Arm1.openClaw();
            }

            if(timeyBoi.time() - arm1InTimer > .45){
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
                    cycle++;
                    cycleFlag = false;
                }
                if(cycle == 6){
                    arm1OutFlag = false;
                }
                if(!arm2UpSlow){
                    Arm2.moveArm(.4);
                }
                if(Arm2.getArmPosition() > 60){
                    Arm2.closeServo();
                }
                if(Arm2.getArmPosition() > 180 && !arm2UpSlow){
                    arm2UpSlow = true;
                    Arm2.moveArm(0.25);
                }
                if(Arm2.getArmPosition() > 240 && arm2UpSlow){
                    arm2IsUp = true;
                    arm2Up = false;
                    arm2UpSlow = false;
                    Arm2.moveArm(0);
                }
            }

            if(arm2IsUp){
                arm2Timer = timeyBoi.time();
                arm2IsUp = false;
            }

            if(timeyBoi.time() - arm2Timer > .05 && !arm2Down) {
                if(cycle == 6){
                    Arm1.forearmDown();
                }
                arm2Timer = 420;
                arm2Down = true;
                grabFlag = true;
                Arm2.openServo();
            }

            if(arm2Down){
                Arm2.moveArm(-1);
                if(Arm2.getArmPosition() < 200 && grabFlag){
                    if(cycle != 6){
                        grab = true;
                    }
                    if(cycle == 0){
                        cycle++;
                    }
                    grabFlag = false;

                }
                if(Arm2.getArmPosition() < 20){
                    arm2Down = false;
                    arm2IsDown = true;
                    Arm2.moveArm(0);
                    //Arm2.resetArm();
                    if(cycle == 6 ){
                        break;
                    }
                }
            }
        }

        Arm1.forearmUpNoRotate();

        DriveTrain.lightClosed();

        drive.followTrajectorySequence(sumo);

        if(broken){
            Arm2.moveArm(0);
            Arm1.forearmMid();
            while(Arm1.getArmPosition() > 10){
                Arm1.moveArm(-.5);
            }
            Arm1.moveArm(0);
        }

        TrajectorySequence left = drive.trajectorySequenceBuilder(correct.end())
                .lineToLinearHeading(new Pose2d(startPose.getX() + -28, startPose.getY() + 2, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(startPose.getX() + -28, startPose.getY() -22, Math.toRadians(0)))
                .build();

        TrajectorySequence middle = drive.trajectorySequenceBuilder(correct.end())
                .lineToLinearHeading(new Pose2d(startPose.getX() + -28, startPose.getY() + 2, Math.toRadians(0)))
                .build();

        TrajectorySequence right = drive.trajectorySequenceBuilder(correct.end())
                .lineToLinearHeading(new Pose2d(startPose.getX() + -28, startPose.getY() + 2, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(startPose.getX() + -28, startPose.getY() + 22, Math.toRadians(0)))
                .build();

        if(tagOfInterest == null || tagOfInterest.id == LEFT){
            drive.followTrajectorySequence(left);
        }else if(tagOfInterest.id == MIDDLE){
            drive.followTrajectorySequence(middle);
        }else{
            drive.followTrajectorySequence(right);
        }

        if(broken){
            Arm1.forearmUp();
            arm2Timer = timeyBoi.time();
            while(Arm2.getArmPosition() > 10 && timeyBoi.time() - arm2Timer < 1){
                Arm2.moveArm(-.5);
            }
            Arm2.moveArm(0);
        }
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
    }
}