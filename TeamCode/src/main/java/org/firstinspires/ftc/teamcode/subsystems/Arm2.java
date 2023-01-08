package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm2 {
    //Motors
    public static DcMotor arm2Left;
    public static DcMotor arm2Right;

    //Constructor
    public Arm2(){}

    public static void initArm2(HardwareMap hwm){
        //Declare Motors on hardware map
        arm2Left = hwm.get(DcMotor.class, "arm2Left");
        arm2Right = hwm.get(DcMotor.class, "arm2Right");

        arm2Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm2Left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm2Left.setDirection(DcMotorSimple.Direction.FORWARD);
        arm2Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm2Right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm2Right.setDirection(DcMotorSimple.Direction.FORWARD);

    }

    public static void moveArm(double power){
        arm2Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm2Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm2Left.setPower(power);
        arm2Right.setPower(power);
    }

    public static void armTop() {
        arm2Right.setTargetPosition(1080);
        arm2Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm2Right.setPower(.75);
        arm2Left.setTargetPosition(1080);
        arm2Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm2Left.setPower(.75);
    }

    public static void armTopTele() {
        arm2Right.setTargetPosition(815);
        arm2Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm2Right.setPower(.75);
        arm2Left.setTargetPosition(815);
        arm2Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm2Left.setPower(.75);
    }

    public static void armMid() {
        arm2Right.setTargetPosition(465);
        arm2Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm2Right.setPower(.98);
        arm2Left.setTargetPosition(465);
        arm2Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm2Left.setPower(.98);
    }

    public static void armLow() {
        arm2Right.setTargetPosition(230);
        arm2Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm2Right.setPower(.75);
        arm2Left.setTargetPosition(230);
        arm2Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm2Left.setPower(.75);
    }

    public static void armTopKinda() {
        arm2Right.setTargetPosition(1000);
        arm2Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm2Right.setPower(-.25);
        arm2Left.setTargetPosition(1000);
        arm2Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm2Left.setPower(-.25);
    }

    public static void armSpecUp(int ticks){
        arm2Right.setTargetPosition(ticks);
        arm2Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm2Right.setPower(.75);
        arm2Left.setTargetPosition(ticks);
        arm2Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm2Left.setPower(.75);
    }

    public static void armSpecDown(int ticks){
        arm2Right.setTargetPosition(ticks);
        arm2Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm2Right.setPower(-.4);
        arm2Left.setTargetPosition(ticks);
        arm2Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm2Left.setPower(-.4);
    }

    public static void armDown(){
        arm2Right.setTargetPosition(0);
        arm2Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm2Right.setPower(-0.75);
        arm2Left.setTargetPosition(0);
        arm2Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm2Left.setPower(-0.75);
    }

}
