package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.vision.NormalizationDemoPipelineConeBlue;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@Autonomous
public class ConeTracking extends LinearOpMode {

    OpenCvWebcam camera;
    NormalizationDemoPipelineConeBlue pipeline;

    public static DcMotorEx leftFront;
    public static DcMotorEx leftBack;
    public static DcMotorEx rightFront;
    public static DcMotorEx rightBack;

    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain.initDriveTrain(hardwareMap);

        // Initialize camera and servo
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "camera2");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new NormalizationDemoPipelineConeBlue(telemetry);

        camera.setPipeline(pipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Init Error", errorCode);
                telemetry.update();

            }
        });

        FtcDashboard.getInstance().startCameraStream(camera, 0);

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

        boolean up = false;
        boolean down = false;

        ElapsedTime timeyBoi = new ElapsedTime();

        timeyBoi.reset();

        waitForStart();

        while(!isStopRequested()){
            /*if(pipeline.getXContour() == 0 && search){
                search = false;
            }
            if(pipeline.getXContour() != 0 && !search){
                search = true;
            }*/

            camCycle++;
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
        }

        camera.closeCameraDevice();
    }

}
