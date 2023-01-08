package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
public class Arm1 {
    //Motors
    public static DcMotor arm1Left;
    public static DcMotor arm1Right;

    //Servos
    public static Servo claw;
    public static Servo rotater;

    public static Servo lifter1;
    public static Servo lifter2;

    //Sensors
    public static DistanceSensor poleSensor;

    //Constants
    private static final double CLAW_OPEN = 0;
    private static final double CLAW_CLOSED = 0.38;
    private static final double ROTATER_UP = 0.16;
    private static final double ROTATER_DOWN = 0.82;
    private static final double LIFTER1_UP = 0.77;
    private static final double LIFTER2_UP = 0.23;
    private static final double LIFTER1_DOWN = 0.05;
    private static final double LIFTER2_DOWN = 0.95;

    //Constructor
    public Arm1(){}

    public static void initArm1(HardwareMap hwm){
        //Declare Motors on hardware map
        arm1Left = hwm.get(DcMotor.class, "arm1Left");
        arm1Right = hwm.get(DcMotor.class, "arm1Right");

        claw = hwm.get(Servo.class, "claw");
        rotater = hwm.get(Servo.class, "rotater");

        lifter1 = hwm.get(Servo.class, "lifter1");
        lifter2 = hwm.get(Servo.class, "lifter2");

        poleSensor = hwm.get(DistanceSensor.class, "poleSensor");

        arm1Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm1Left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm1Left.setDirection(DcMotorSimple.Direction.REVERSE);
        arm1Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm1Right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm1Right.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public static void forearmUp(){
        lifter1.setPosition(LIFTER1_UP);
        lifter2.setPosition(LIFTER2_UP);
        rotater.setPosition(ROTATER_UP);
    }

    public static void forearmDown(){
        lifter1.setPosition(LIFTER1_DOWN);
        lifter2.setPosition(LIFTER2_DOWN);
        rotater.setPosition(ROTATER_DOWN);
    }

    public static void clawOpen(){
        claw.setPosition(CLAW_OPEN);
    }

    public static void clawClosed(){
        claw.setPosition(CLAW_CLOSED);
    }

    public static void moveArm(double power){
        arm1Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm1Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm1Left.setPower(power);
        arm1Right.setPower(power);
    }

    public static void armTop() {
        arm1Right.setTargetPosition(1080);
        arm1Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm1Right.setPower(.75);
        arm1Left.setTargetPosition(1080);
        arm1Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm1Left.setPower(.75);
    }

    public static void armTopTele() {
        arm1Right.setTargetPosition(815);
        arm1Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm1Right.setPower(.98);
        arm1Left.setTargetPosition(815);
        arm1Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm1Left.setPower(.98);
    }

    public static void armMid() {
        arm1Right.setTargetPosition(465);
        arm1Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm1Right.setPower(.98);
        arm1Left.setTargetPosition(465);
        arm1Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm1Left.setPower(.98);
    }

    public static void armLow() {
        arm1Right.setTargetPosition(230);
        arm1Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm1Right.setPower(.75);
        arm1Left.setTargetPosition(230);
        arm1Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm1Left.setPower(.75);
    }

    public static void armTopKinda() {
        arm1Right.setTargetPosition(1000);
        arm1Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm1Right.setPower(-.25);
        arm1Left.setTargetPosition(1000);
        arm1Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm1Left.setPower(-.25);
    }

    public static void armSpecUp(int ticks){
        arm1Right.setTargetPosition(ticks);
        arm1Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm1Right.setPower(.75);
        arm1Left.setTargetPosition(ticks);
        arm1Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm1Left.setPower(.75);
    }

    public static void armSpecDown(int ticks){
        arm1Right.setTargetPosition(ticks);
        arm1Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm1Right.setPower(-.4);
        arm1Left.setTargetPosition(ticks);
        arm1Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm1Left.setPower(-.4);
    }

    public static void armDown(){
        arm1Right.setTargetPosition(0);
        arm1Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm1Right.setPower(-0.75);
        arm1Left.setTargetPosition(0);
        arm1Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm1Left.setPower(-0.75);
    }

}
