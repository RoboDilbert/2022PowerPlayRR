package org.firstinspires.ftc.teamcode.subsystems;

import androidx.lifecycle.LifecycleEventObserver;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
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
    public static DistanceSensor clawSensor;

    //LEDs
    public static RevBlinkinLedDriver blinkinLedDriver;

    //Constants
    private static final double CLAW_OPEN = 0.4;
    private static final double CLAW_HALF = 0.5;
    private static final double CLAW_CLOSED = 0.66;
    private static final double ROTATER_UP = 0.845;
    private static final double ROTATER_DOWN = 0.145;
    private static final double LIFTER1_UP = 0.73;
    private static final double LIFTER2_UP = 0.27;
    private static final double LIFTER1_DOWN = 0.05;
    private static final double LIFTER2_DOWN = 0.95;

    //States
    private static ARM1_STATE currentArm1State = ARM1_STATE.IN;
    private static LIFTER_STATE currentLifterState = LIFTER_STATE.UP;
    private static CLAW_STATE currentClawState = CLAW_STATE.OPEN;
    private static ROTATER_STATE currentRotaterState = ROTATER_STATE.UP;

    private enum ARM1_STATE{
        IN,
        OUT;
    }
    private enum LIFTER_STATE{
        UP,
        DOWN;
    }
    private enum CLAW_STATE{
        OPEN,
        CLOSED;
    }
    private enum ROTATER_STATE{
        UP,
        DOWN;
    }

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

        clawSensor = hwm.get(DistanceSensor.class, "clawSensor");

        blinkinLedDriver = hwm.get(RevBlinkinLedDriver.class, "blinkin");

        arm1Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm1Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm1Left.setDirection(DcMotorSimple.Direction.REVERSE);
        arm1Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm1Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm1Right.setDirection(DcMotorSimple.Direction.REVERSE);

        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE);//AQUA

        //forearmUp();
    }

    public static void forearmUp(){
        lifter1.setPosition(LIFTER1_UP);
        lifter2.setPosition(LIFTER2_UP);
        rotater.setPosition(ROTATER_UP);
        lifterChangeState("UP");
        rotaterChangeState("UP");
    }

    public static void forearmUpNoRotate(){
        lifter1.setPosition(LIFTER1_UP);
        lifter2.setPosition(LIFTER2_UP);
        lifterChangeState("UP");
    }

    public static void forearmShort(){
        lifter1.setPosition(0.28);
        lifter2.setPosition(0.72);
        rotater.setPosition(ROTATER_DOWN);
        lifterChangeState("DOWN");
        rotaterChangeState("DOWN");
    }

    public static void forearmDown(){
        lifter1.setPosition(LIFTER1_DOWN);
        lifter2.setPosition(LIFTER2_DOWN);
        rotater.setPosition(ROTATER_DOWN);
        lifterChangeState("DOWN");
        rotaterChangeState("DOWN");
    }

    public static void forearmSpecDown(double pos){
        lifter1.setPosition(1 - pos);
        lifter2.setPosition(pos);
        rotater.setPosition(ROTATER_DOWN);
        lifterChangeState("DOWN");
        rotaterChangeState("DOWN");
    }

    public static void openClaw(){
        claw.setPosition(CLAW_OPEN);
        clawChangeState("OPEN");
    }

    public static void closeClaw(){
        claw.setPosition(CLAW_CLOSED);
        clawChangeState("CLOSED");
    }

    public static void rotaterUp(){
        rotater.setPosition(ROTATER_UP);
        rotaterChangeState("UP");
    }

    public static void resetArm(){
        arm1Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm1Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm1Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm1Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public static int getArmPosition(){return (arm1Left.getCurrentPosition() + arm1Right.getCurrentPosition()) / 2;}

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
        arm1Right.setTargetPosition(435);
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


    public static void arm1ChangeState(String state){
        if(state.equals("IN")){
            currentArm1State = ARM1_STATE.IN;
        }
        else if(state.equals("OUT")){
            currentArm1State = ARM1_STATE.OUT;
        }
    }

    public static void lifterChangeState(String state){
        if(state.equals("UP")){
            currentLifterState = LIFTER_STATE.UP;
        }
        else if(state.equals("DOWN")){
            currentLifterState = LIFTER_STATE.DOWN;
        }
    }

    public static void clawChangeState(String state){
        if(state.equals("OPEN")){
            currentClawState = CLAW_STATE.OPEN;
        }
        else if(state.equals("CLOSED")){
            currentClawState = CLAW_STATE.CLOSED;
        }
    }

    public static void rotaterChangeState(String state){
        if(state.equals("UP")){
            currentRotaterState = ROTATER_STATE.UP;
        }
        else if(state.equals("DOWN")){
            currentRotaterState = ROTATER_STATE.DOWN;
        }
    }

    public ARM1_STATE getArm1State(){return currentArm1State;}
    public LIFTER_STATE getLifterState(){return currentLifterState;}
    public CLAW_STATE getClawState(){return currentClawState;}
    public ROTATER_STATE getRotaterState(){return currentRotaterState;}
}
