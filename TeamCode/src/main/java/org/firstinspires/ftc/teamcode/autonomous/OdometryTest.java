package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.odometry.OdometryGlobalCoordinatePosition;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

@Autonomous(name= "OdoTest", group= "Autonomous")

public class OdometryTest extends LinearOpMode {

    OdometryGlobalCoordinatePosition globalPositionUpdate;

    static final double COUNTS_PER_INCH = 1300.30;

    double previousTargetX = 0;
    double getPreviousTargetY = 0;

    public void runOpMode() throws InterruptedException {

        DriveTrain.initDriveTrain(hardwareMap);
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(DriveTrain.leftWheel, DriveTrain.rightWheel, DriveTrain.middleWheel, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        DriveTrain.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DriveTrain.leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DriveTrain.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DriveTrain.rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        goToPosition(0,51 * COUNTS_PER_INCH, 0.4, 0, 1 * COUNTS_PER_INCH);//0.6

        sleep(100);

        turn(90);

        sleep(100);

        globalPositionUpdate.resetEncoders();

        goToPosition(0,-8 * COUNTS_PER_INCH, 0.3, 0, 1 * COUNTS_PER_INCH);//0.6

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

        double feedForward = 0.1;
        double xMultiplier = 1.1;
        double yMultiplier = 1.5;

        boolean turnFlag = false;

        while(distance > allowedError || pivot > (Math.PI)/180){
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
                DriveTrain.leftFront.setPower ((robotMovementYComponent * ((distance/initialDistance) * yMultiplier) + (xMultiplier * (distance/initialDistance + .4)* robotMovementXComponent)) + pivotPower); //+ feedForward
                DriveTrain.rightFront.setPower((robotMovementYComponent * ((distance/initialDistance) * yMultiplier) - (xMultiplier * (distance/initialDistance + .4)* robotMovementXComponent)) - pivotPower);// + feedForward
                DriveTrain.leftBack.setPower  ((robotMovementYComponent * ((distance/initialDistance) * yMultiplier) - (xMultiplier * (distance/initialDistance + .4)* robotMovementXComponent)) + pivotPower); //+ feedForward
                DriveTrain.rightBack.setPower ((robotMovementYComponent * ((distance/initialDistance) * yMultiplier) + (xMultiplier * (distance/initialDistance + .4)* robotMovementXComponent)) - pivotPower); //+ feedForward
                turnFlag = false;
            }

            previousAngle = Math.toRadians(desiredOrientation - globalPositionUpdate.returnOrientation());
            previousDistance = distance;

            telemetry.addData("Distance To X", distanceToXTarget);
            telemetry.addData("Distance To Y", distanceToYTarget);
            telemetry.addData("Hypotenuse Distance", distance);
            telemetry.update();
        }

        previousTargetX = targetXPos;
        getPreviousTargetY= targetYPos;

        DriveTrain.stop();
        sleep(250);
    }

    public void turn(double desiredOrientation){
        double pivot = Math.toRadians(desiredOrientation - globalPositionUpdate.returnOrientation());

        double currentOrientation = Math.toRadians(globalPositionUpdate.returnOrientation());

        double pivotPower = 0;
        double pivotMultiplier = 1.25;
        double pivotFf = 0.085;

        while(Math.abs(currentOrientation) < pivot - Math.PI / 90 || Math.abs(currentOrientation) > pivot + Math.PI / 90){
            pivot = Math.toRadians(desiredOrientation - globalPositionUpdate.returnOrientation());
            currentOrientation = Math.toRadians(globalPositionUpdate.returnOrientation());
            if(Math.abs(pivot) > (Math.PI / 18)){
                pivotPower = 0.20;
            }
            else{
                if(Math.abs(pivot) > (Math.PI / 180)){
                    pivotPower = (Math.abs(pivot / 0.7) * pivotMultiplier) + pivotFf;
                }
                else{
                    pivotPower = 0;
                }
            }
            if(pivot < 0){
                pivotPower = pivotPower * -1;
            }
            DriveTrain.leftFront.setPower(pivotPower);
            DriveTrain.rightFront.setPower(-pivotPower);
            DriveTrain.leftBack.setPower(pivotPower);
            DriveTrain.rightBack.setPower(-pivotPower);
        }
        DriveTrain.stop();
    }

    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }
}
