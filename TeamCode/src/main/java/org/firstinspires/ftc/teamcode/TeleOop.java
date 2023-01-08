package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Arm1;
import org.firstinspires.ftc.teamcode.subsystems.Arm2;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

@TeleOp(name="TeleOop", group="Linear Opmode")

public class TeleOop extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        Arm1.initArm1(hardwareMap);
        Arm2.initArm2(hardwareMap);
        DriveTrain.initDriveTrain(hardwareMap);

        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.left_bumper)
                DriveTrain.mecanumDrive(-gamepad1.left_stick_y * 0.35, gamepad1.left_stick_x * 0.45, gamepad1.right_stick_x * 0.35);
            else
                DriveTrain.mecanumDrive(-gamepad1.left_stick_y * 0.55, gamepad1.left_stick_x * 0.55, gamepad1.right_stick_x * 0.35);

        }
    }
}
