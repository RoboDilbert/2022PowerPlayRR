package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.odometry.OdometryGlobalCoordinatePosition;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


import java.util.ArrayList;

@Autonomous(name= "OdoTest", group= "Autonomous")

public class OdometryTest extends LinearOpMode {

    OdometryGlobalCoordinatePosition globalPositionUpdate;

    static final double COUNTS_PER_INCH = 1300.30;

    double previousTargetX = 0;
    double getPreviousTargetY = 0;

    OpenCvCamera camera;
    org.firstinspires.ftc.teamcode.AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

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

    public void runOpMode() throws InterruptedException {

        DriveTrain.initDriveTrain(hardwareMap);
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(DriveTrain.leftWheel, DriveTrain.rightWheel, DriveTrain.middleWheel, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        DriveTrain.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DriveTrain.leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DriveTrain.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DriveTrain.rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new org.firstinspires.ftc.teamcode.AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

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

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        waitForStart();

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        goToPosition(0,52 * COUNTS_PER_INCH, 0.3, 0, 0 * COUNTS_PER_INCH);//0.6

        sleep(100);

        /*globalPositionUpdate.resetEncoders();
        DriveTrain.setRunMode("RUN_WITHOUT_ENCODER");
        DriveTrain.leftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DriveTrain.rightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DriveTrain.middleWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);*/

        turn(90);

        Orientation angles = DriveTrain.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Current orientation: ", globalPositionUpdate.returnOrientation());
        telemetry.addData("Current gyro orientation: ", angles.firstAngle);
        telemetry.update();

        sleep(2000);

        /* Actually do something useful */
        if(tagOfInterest == null || tagOfInterest.id == LEFT){
            globalPositionUpdate.resetEncoders();
            DriveTrain.setRunMode("RUN_WITHOUT_ENCODER");
            DriveTrain.leftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            DriveTrain.rightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            DriveTrain.middleWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            goToPosition(0,-24 * COUNTS_PER_INCH, -0.3, 0, 1 * COUNTS_PER_INCH);//0.6

            sleep(100);
        }else if(tagOfInterest.id == MIDDLE){
            //trajectory
        }else{
            globalPositionUpdate.resetEncoders();
            DriveTrain.setRunMode("RUN_WITHOUT_ENCODER");
            DriveTrain.leftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            DriveTrain.rightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            DriveTrain.middleWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            goToPosition(0,24 * COUNTS_PER_INCH, 0.3, 0, 1 * COUNTS_PER_INCH);//0.6

            sleep(100);
        }
    }

    public void goToPosition(double targetXPos, double targetYPos, double power, double desiredOrientation, double allowedError){

        double distanceToXTarget = targetXPos - globalPositionUpdate.returnXCoordinate();
        double distanceToYTarget = targetYPos - globalPositionUpdate.returnYCoordinate();

        double distance = Math.hypot(distanceToXTarget, distanceToYTarget);
        double previousDistance = 0;
        double previousAngle = 0;

        telemetry.addData("Distance To X", distanceToXTarget);
        telemetry.addData("Distance To Y", distanceToYTarget);
        telemetry.addData("Hypotenuse Distance", distance);
        telemetry.update();

        double initialDistance = Math.hypot(distanceToXTarget, distanceToYTarget);

        double pivotPower = 0;
        double pivotMultiplier = 1.25;
        double pivotFf = 0.06;

        double pivot = Double.MAX_VALUE;

        double feedForward = 0.13;
        double xMultiplier = 1.1;
        double yMultiplier = 1.4;

        boolean turnFlag = false;

        while(distanceToYTarget > allowedError || pivot > (Math.PI)/90){
            distanceToXTarget = targetXPos - globalPositionUpdate.returnXCoordinate();
            distanceToYTarget = targetYPos - globalPositionUpdate.returnYCoordinate();
            distance = Math.hypot(distanceToXTarget, distanceToYTarget);

            double robotMovementAngle = Math.toDegrees(Math.atan2(distanceToXTarget, distanceToYTarget));
            double PercentOfTarget = (distance/initialDistance);

            double robotMovementXComponent = calculateX(robotMovementAngle, power);
            double robotMovementYComponent = calculateY(robotMovementAngle, power);
            pivot = Math.toRadians(desiredOrientation - globalPositionUpdate.returnOrientation());

            /*if(Math.abs(distance - previousDistance) < 15){
                xMultiplier += 0.02;
                yMultiplier += 0.04;
            }*/
            /*if(Math.abs(pivot) > Math.PI / 180 && Math.abs(previousAngle - pivot) < Math.PI/90){
                pivotMultiplier += 0.01;
            }*/
            if(Math.abs(previousAngle - pivot) < Math.PI/360 || Math.abs(previousAngle - pivot) > Math.PI/90){
                pivotMultiplier = 1.25;
            }

            if(Math.abs(pivot) > (Math.PI / 18)){
                pivotPower = 0.40;
            }
            else{
                if(Math.abs(pivot) > (Math.PI / 90)){
                    pivotPower = (Math.abs(pivot / 0.7) * pivotMultiplier) + pivotFf;
                }
                else{
                    pivotPower = 0;
                }
            }
            if(pivot < 0){
                pivotPower = pivotPower * -1;
            }

            if(distance/initialDistance <= .5)
                robotMovementYComponent = robotMovementYComponent * (distance/initialDistance) + feedForward;

            /*if((targetXPos == previousTargetX && targetYPos == getPreviousTargetY) ||  pivot > (Math.PI)/6){
                DriveTrain.leftFront.setPower(pivotPower);
                DriveTrain.rightFront.setPower(-pivotPower);
                DriveTrain.leftBack.setPower(pivotPower);
                DriveTrain.rightBack.setPower(-pivotPower);
                distance = 0;
                turnFlag = true;
            }*/
            //pivotPower = 0;
            if(targetXPos == 69.696969696969){

            }
            else{
                DriveTrain.leftFront.setPower ((robotMovementYComponent) + (xMultiplier * (distance/initialDistance + .4)* robotMovementXComponent) + pivotPower); //+ feedForward
                DriveTrain.rightFront.setPower((robotMovementYComponent) - (xMultiplier * (distance/initialDistance + .4)* robotMovementXComponent) - pivotPower);// + feedForward
                DriveTrain.leftBack.setPower  ((robotMovementYComponent) - (xMultiplier * (distance/initialDistance + .4)* robotMovementXComponent) + pivotPower); //+ feedForward
                DriveTrain.rightBack.setPower ((robotMovementYComponent) + (xMultiplier * (distance/initialDistance + .4)* robotMovementXComponent) - pivotPower); //+ feedForward
                turnFlag = false;
            }

            previousAngle = Math.toRadians(desiredOrientation - globalPositionUpdate.returnOrientation());
            previousDistance = distance;

            telemetry.addData("Left Front: ", DriveTrain.leftFront.getMode());
            telemetry.addData("Right Back: ", DriveTrain.rightBack.getMode());
            telemetry.addData("Distance To X", distanceToXTarget);
            telemetry.addData("Distance To Y", distanceToYTarget);
            telemetry.addData("Hypotenuse Distance", distance);
            telemetry.update();
        }

        previousTargetX = targetXPos;
        getPreviousTargetY= targetYPos;


        //DriveTrain.stop();
        DriveTrain.autoBrake(500);
        sleep(250);
    }

    public void turn(double desiredOrientation){
        double pivot;

        double currentOrientation = Math.toRadians(globalPositionUpdate.returnOrientation());

        double pivotPower = 0;

        while(Math.abs(currentOrientation) < Math.abs(Math.toRadians(desiredOrientation)) - Math.PI / 360 ){
            pivot = Math.toRadians(desiredOrientation - globalPositionUpdate.returnOrientation());
            currentOrientation = Math.toRadians(globalPositionUpdate.returnOrientation());
            if(Math.abs(pivot) > (Math.PI / 9)){
                pivotPower = 0.25;
            }
            else{
                pivotPower = 0.15;
            }
            if(pivot < 0){
                pivotPower = pivotPower * -1;
            }
            DriveTrain.leftFront.setPower(pivotPower);
            DriveTrain.rightFront.setPower(-pivotPower);
            DriveTrain.leftBack.setPower(pivotPower);
            DriveTrain.rightBack.setPower(-pivotPower);

            telemetry.addData("Pivot: ", pivot);
            telemetry.update();
        }
        DriveTrain.stop();
    }

    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
    }
}
