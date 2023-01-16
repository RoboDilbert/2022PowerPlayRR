package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.Arm1;
import org.firstinspires.ftc.teamcode.subsystems.Arm2;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

@Disabled
@TeleOp(name="TeleOop2", group="Linear Opmode")

public class yourmom extends LinearOpMode {
    private boolean autoDrop = false;
    private boolean armManual = false;
    private boolean armStop = false;
    private boolean autoClose = false;
    private boolean cone = false;
    private boolean armDown = true;
    private boolean armReset = false;
    public void runOpMode() throws InterruptedException {
        Arm1.initArm1(hardwareMap);
        Arm2.initArm2(hardwareMap);
        DriveTrain.initDriveTrain(hardwareMap);

        waitForStart();

        Arm1.lifter1.setPosition(.75);
        Arm1.lifter2.setPosition(.25);
        Arm1.rotater.setPosition(.16);
        Arm1.claw.setPosition(.38);

        while(opModeIsActive()){
            /*if(gamepad1.left_bumper)
                DriveTrain.mecanumDrive(-gamepad1.left_stick_y * 0.35, gamepad1.left_stick_x * 0.45, gamepad1.right_stick_x * 0.35);
            else
                DriveTrain.mecanumDrive(-gamepad1.left_stick_y * 0.55, gamepad1.left_stick_x * 0.55, gamepad1.right_stick_x * 0.35);

            if(gamepad2.y) {
                Arm2.armTopTele();
                armDown = false;
                armReset = false;
            }
            if(gamepad2.x) {
                Arm1.armMid();
                armDown = false;
                armReset = false;
            }
            if(gamepad2.b) {
                Arm1.armLow();
                armDown = false;
                armReset = false;
            }
            if(gamepad2.a) {
                Arm2.armDown();
                armDown = true;
                autoDrop = false;
                armReset = false;
            }


            if(gamepad2.right_stick_y <= -.1){
                Arm2.moveArm(-gamepad2.right_stick_y * 0.65);
            }
            else if(gamepad2.right_stick_y >= .1)
                Arm2.moveArm(-gamepad2.right_stick_y * 0.65);
            else
                Arm2.moveArm(0);



            if(gamepad2.left_stick_y <= -.1){
                Arm1.moveArm(-gamepad2.left_stick_y * 0.65);
            }
            else if(gamepad2.left_stick_y >= .1)
                Arm1.moveArm(-gamepad2.left_stick_y * 0.65);
            else
                Arm1.moveArm(0);


            if(gamepad1.x){
                Arm1.claw.setPosition(.38);
            }
            if(gamepad1.b){
                Arm1.claw.setPosition(0);
            }
            if(gamepad1.y){
                Arm1.lifter1.setPosition(.77);
                Arm1.lifter2.setPosition(.23);
                Arm1.rotater.setPosition(.16);
            }
            if(gamepad1.a){
                Arm1.lifter1.setPosition(.05);
                Arm1.lifter2.setPosition(.95);
                Arm1.rotater.setPosition(.82);
            }


            telemetry.addData("Left: ", DriveTrain.getLeftPosition());
            telemetry.addData("Right: ", DriveTrain.getRightPosition());
            telemetry.addData("Middle: ", DriveTrain.getMiddlePosition());

            /*telemetry.addData("Right: ", DriveTrain.getRightPosition());
            telemetry.addData("Middle: ", DriveTrain.getMiddlePosition());
            telemetry.addData("Arm1 Height Left: ", Arm1.arm1Left.getCurrentPosition());
            telemetry.addData("Arm1 Height Right: ", Arm1.arm1Right.getCurrentPosition());
            telemetry.addData("Arm2 Height Left: ", Arm2.arm2Left.getCurrentPosition());
            telemetry.addData("Arm2 Height Right: ", Arm2.arm2Right.getCurrentPosition());*/

        }
    }
}
