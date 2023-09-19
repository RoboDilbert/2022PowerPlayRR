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
    int YLower = 10;
    int CrLower = 175;
    int CbLower = 60;
    int YUpper = 215;
    int CrUpper = 255;
    int CbUpper = 130;

    boolean yUpUp = false;
    boolean yUpDown = false;
    boolean yDownUp = false;
    boolean yDownDown = false;
    boolean crUpUp = false;
    boolean crUpDown = false;
    boolean crDownUp = false;
    boolean crDownDown = false;
    boolean cbUpUp = false;
    boolean cbUpDown = false;
    boolean cbDownUp = false;
    boolean cbDownDown = false;

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
                camera.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
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

        FtcDashboard.getInstance().startCameraStream(camera, 0);

        while(!isStarted() && !isStopRequested()){
            if(gamepad1.y && !yUpUp){yUpUp = true;}
            if(!gamepad1.y && yUpUp){
                yUpUp = false;
                pipeline.incYMax();
            }
            if(gamepad1.a && !yUpDown){yUpDown = true;}
            if(!gamepad1.a && yUpDown){
                yUpDown = false;
                pipeline.decYMax();
            }
            if(gamepad1.b && !yDownUp){yDownUp = true;}
            if(!gamepad1.b && yDownUp){
                yDownUp = false;
                pipeline.incYMin();
            }
            if(gamepad1.x && !yDownDown){yDownDown = true;}
            if(!gamepad1.x && yDownDown){
                yDownDown = false;
                pipeline.decYMin();
            }

            if(gamepad2.y && !crUpUp){crUpUp = true;}
            if(!gamepad2.y && crUpUp){
                crUpUp = false;
                pipeline.incCrMax();
            }
            if(gamepad2.a && !crUpDown){crUpDown = true;}
            if(!gamepad2.a && crUpDown){
                crUpDown = false;
                pipeline.decCrMax();
            }
            if(gamepad2.b && !crDownUp){crDownUp = true;}
            if(!gamepad2.b && crDownUp){
                crDownUp = false;
                pipeline.incCrMin();
            }
            if(gamepad2.x && !crDownDown){crDownDown = true;}
            if(!gamepad2.x && crDownDown){
                crDownDown = false;
                pipeline.decCrMin();
            }

            if(gamepad2.dpad_up && !cbUpUp){cbUpUp = true;}
            if(!gamepad2.dpad_up && cbUpUp){
                cbUpUp = false;
                pipeline.incCbMax();
            }
            if(gamepad2.dpad_down && !cbUpDown){cbUpDown = true;}
            if(!gamepad2.dpad_down && cbUpDown){
                cbUpDown = false;
                pipeline.decCbMax();
            }
            if(gamepad2.dpad_right && !cbDownUp){cbDownUp = true;}
            if(!gamepad2.dpad_right && cbDownUp){
                cbDownUp = false;
                pipeline.incCbMin();
            }
            if(gamepad2.dpad_left && !cbDownDown){cbDownDown = true;}
            if(!gamepad2.dpad_left && cbDownDown){
                cbDownDown = false;
                pipeline.decCbMin();
            }
        }

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
