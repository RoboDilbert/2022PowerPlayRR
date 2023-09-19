package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp (name="BrewerBot", group="TeleOp")
public class BrewersBot extends LinearOpMode {

    //Declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx left = null;
    private DcMotorEx right = null;


    private Servo flip = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Init motors
        left = hardwareMap.get(DcMotorEx.class, "left");
        right = hardwareMap.get(DcMotorEx.class, "right");

        flip = hardwareMap.get(Servo.class, "flip");

        //We set the directions for each motor.
        left.setDirection(DcMotor.Direction.REVERSE);
        right.setDirection(DcMotor.Direction.FORWARD);

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {


            double leftDrive = gamepad1.right_stick_y;
            double rightPower = gamepad1.left_stick_y;
            double leftPower = Range.clip(leftDrive, -1, 1);

            left.setPower(leftPower);
            right.setPower(rightPower);

            if(gamepad1.x && gamepad1.b){
                flip.setPosition(0.5);
            }
            if(gamepad1.a && gamepad1.y) {
                flip.setPosition(0.1);
            }
           /* if(gamepad1.left_stick_y > .75 && gamepad1.right_stick_y >.75){
                left.setVelocity(500);
                right.setVelocity(500);
            } else if (gamepad1.left_stick_y <.9&& gamepad1.right_stick_y < 0.9) {
                left.setVelocity(0);
                right.setVelocity(0);
            }*/

            telemetry.update();


        }

    }
}
