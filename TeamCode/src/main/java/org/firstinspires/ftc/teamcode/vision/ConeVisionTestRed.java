package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@Autonomous
public class ConeVisionTestRed extends LinearOpMode {

    OpenCvWebcam camera;
    NormalizationDemoPipelineConeRed pipeline;

    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain.initDriveTrain(hardwareMap);

        // Initialize camera and servo
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "camera2");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new NormalizationDemoPipelineConeRed(telemetry);

        camera.setPipeline(pipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 176, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Init Error", errorCode);
                telemetry.update();

            }
        });

        FtcDashboard.getInstance().startCameraStream(camera, 0);


        waitForStart();

        while(!isStopRequested()){
            telemetry.addData("X Pos: ", pipeline.getXContour());
            telemetry.update();
        }

        camera.closeCameraDevice();
    }
    public void correctDrive(){
        int target = 159;
        while((pipeline.getXContour() < target - 1 || pipeline.getXContour() > target + 1) && !isStopRequested()){
            if(pipeline.getXContour() < target){
                DriveTrain.turn(-0.3);
            }
            else if(pipeline.getXContour() > target){
                DriveTrain.turn(0.3);
            }
        }
        DriveTrain.turn(0);
    }

}
