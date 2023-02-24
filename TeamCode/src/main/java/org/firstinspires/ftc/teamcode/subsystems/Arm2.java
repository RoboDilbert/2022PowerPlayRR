package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm2 {
    //Motors
    public static DcMotor arm2Left;
    public static DcMotor arm2Right;

    //Servos
    public static Servo sombrero;

    //Constants
    private static final double OPEN = 0.15;
    private static final double CLOSED = 0.5;

    //Constructor
    public Arm2(){}

    public static void initArm2(HardwareMap hwm){
        //Declare Motors on hardware map
        arm2Left = hwm.get(DcMotor.class, "arm2Left");
        arm2Right = hwm.get(DcMotor.class, "arm2Right");

        sombrero = hwm.get(Servo.class, "sombrero");

        arm2Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm2Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm2Left.setDirection(DcMotorSimple.Direction.FORWARD);
        arm2Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm2Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm2Right.setDirection(DcMotorSimple.Direction.FORWARD);

    }

    public static void resetArm(){
        arm2Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm2Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm2Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm2Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public static int getArmPosition(){return (arm2Left.getCurrentPosition() + arm2Right.getCurrentPosition()) / 2;}

    public static void moveArm(double power){
        arm2Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm2Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm2Left.setPower(power);
        arm2Right.setPower(power);
    }

    public static void armDown(){
        arm2Right.setTargetPosition(0);
        arm2Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm2Right.setPower(-1);
        arm2Left.setTargetPosition(0);
        arm2Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm2Left.setPower(-1);
    }

    public static void openServo(){sombrero.setPosition(OPEN);}

    public static void closeServo(){sombrero.setPosition(CLOSED);}
}
