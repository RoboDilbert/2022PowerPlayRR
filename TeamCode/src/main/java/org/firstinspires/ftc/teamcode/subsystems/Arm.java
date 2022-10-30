package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
public class Arm {
    //Motors
    //public static DcMotor arm;

    //Servos
    public static Servo claw;

    //Constructor
    public Arm(){}

    public static void initArm(HardwareMap hwm){
        //Declare Motors on hardware map
        //arm = hwm.get(DcMotor.class, "arm");
        //claw = hwm.get(Servo.class, "claw");

        //arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //arm.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public static void moveArm(double power){
        //arm.setPower(power);
    }

    public static void openClaw(){
        claw.setPosition(.7);
    }

    public static void closeClaw(){
        claw.setPosition(.4);
    }
}
