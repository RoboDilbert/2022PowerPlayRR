package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.Arm1;
import org.firstinspires.ftc.teamcode.subsystems.Arm2;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

@TeleOp(name="TeleOop", group="Linear Opmode")

public class TeleOop extends LinearOpMode {
    ElapsedTime timeyBoi = new ElapsedTime();

    public static boolean auto = false;
    public static boolean autoCone = false;
    public static boolean dpad = false;
    public static boolean searchCone = false;
    public static boolean autonomous = false;
    public static boolean autoFlag = false;

    public static boolean arm1Out = false;
    public static boolean arm1OutFlag = true;
    public static double senseConeTimer = 0;
    public static boolean senseCone = false;
    public static boolean arm1IsOut = false;
    public static boolean haveCone = false;
    public static double arm1OutTimer = 420;
    public static double coneTimer = 420;
    public static boolean arm1In = false;
    public static boolean arm1IsIn = false;
    public static boolean arm1InTimerFlag = false;
    public static double arm1InTimer = 420;
    public static boolean arm2Up = false;
    public static boolean arm2UpSlow = false;
    public static boolean arm2IsUp = false;
    public static double arm2Timer = 420;
    public static boolean arm2Down = false;
    public static boolean arm2IsDown = false;
    public static boolean grab = true;
    public static boolean grabFlag = false;
    public static boolean rotate = false;

    public static boolean cycleFlag = false;
    public static double cycle = 0;

    public static boolean rbumper = false;
    public static boolean rbumperSlow = false;
    public static boolean lbumper = false;
    public static boolean resetArm1 = false;
    public static boolean resetArm2 = false;
    public static boolean armMid = false;
    public static double armTimer = 0;
    public static boolean lookCone = false;
    public static double armDownTimer = 420;
    public static boolean armManualUp = false;
    public static boolean armManualDown = false;
    public static double heightManual = .95;
    public static double totalCurrent = 0.0;
    public static boolean totalFlag = false;
    public static double totalTimer = 0;

    public static boolean fieldCentric = true;


    public void runOpMode() throws InterruptedException {
        Arm1.initArm1(hardwareMap);
        Arm2.initArm2(hardwareMap);
        DriveTrain.initDriveTrain(hardwareMap);

        DriveTrain.lightClosed();

        waitForStart();

        timeyBoi.reset();

        while(opModeIsActive()){

            /**
             * Gamepad1 Controls
             */

            DriveTrain.cartesianDrive((-gamepad1.left_stick_x / 1.4), (gamepad1.left_stick_y / 1.3), (gamepad1.right_stick_x / 2.5), Math.PI);


            if(gamepad1.dpad_right){
                DriveTrain.resetGyro();
            }

            if(gamepad1.dpad_left){
                Arm2.sombrero.setPosition(0.5);
            }

            if(gamepad1.dpad_down){
                Arm2.sombrero.setPosition(0.15);
            }

            if(gamepad1.y){
                auto = true;
                armModes();
            }
            if(gamepad1.a){
                auto = false;
                Arm1.moveArm(0);
                Arm2.moveArm(0);
            }
            if(gamepad1.x){
                resetAuto();
            }
            if(gamepad1.b){
                Arm1.forearmUpNoRotate();
                Arm1.openClaw();
                Arm2.openServo();
                resetArm1 = true;
                resetArm2 = true;
            }
            if(resetArm1 && !auto && !searchCone && !autoCone){
                Arm1.moveArm(-.5);
                if(Arm1.getArmPosition() < 5){
                    resetArm1 = false;
                    Arm1.moveArm(0);
                    Arm1.resetArm();
                }
            }
            if(resetArm2 && !auto && !searchCone && !autoCone){
                Arm2.moveArm(-1);
                if(Arm2.getArmPosition() < 5){
                    resetArm2 = false;
                    Arm2.moveArm(0);
                    Arm2.resetArm();
                }
            }


            if(gamepad1.left_bumper){
                //Arm1.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE);
                DriveTrain.light.setPosition(0);
            }
            if(gamepad1.right_bumper){
                //Arm1.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                DriveTrain.light.setPosition(1);
            }

            if(gamepad1.dpad_up && !autoCone && !dpad){
                dpad = true;
            }
            if(!gamepad1.dpad_up && !autoCone && dpad){
                dpad = false;
                autoCone = true;
            }
            if(gamepad1.dpad_up && autoCone && !dpad){
                dpad = true;
            }
            if(!gamepad1.dpad_up && autoCone && dpad){
                dpad = false;
                autoCone = false;
            }
            if(gamepad1.dpad_down && !autoFlag){
                autoFlag = true;
            }
            if(!gamepad1.dpad_down && autoFlag){
                autoFlag = false;
                autonomous = true;
            }


            if(gamepad1.left_trigger >= .2 && !auto && !searchCone && !autoCone && !lookCone){
                lookCone = true;
            }
            else if(gamepad1.left_trigger < .2 && !auto && !searchCone && !autoCone && lookCone){
                lookCone = false;
            }
            if(lookCone){
                if(Arm1.clawSensor.getDistance(DistanceUnit.CM) < 4){
                    Arm1.closeClaw();
                    lookCone = false;
                }
            }

            /**
             * Gamepad2 Controls
             */

            if(gamepad2.a){
                Arm1.forearmDownNoRotate();
                armMid = false;
                armDownTimer = timeyBoi.time();
            }
            if(timeyBoi.time() > armDownTimer + 0.4){
                armDownTimer = 420;
                Arm1.forearmDown();
            }
            if(gamepad2.b){
                Arm1.closeClaw();
            }
            if(gamepad2.x){
                Arm1.openClaw();
                /*if(armMid){
                    Arm1.forearmMid();
                    armMid = false;
                }*/
            }
            if(gamepad2.y){
                Arm1.forearmUp();
                armMid = false;
            }


            if(gamepad2.right_stick_y <= -.1 && !auto && !searchCone && !autoCone){
                Arm2.moveArm(-gamepad2.right_stick_y);
            }
            else if(gamepad2.right_stick_y >= .1 && !auto && !searchCone && !autoCone){
                Arm2.moveArm(-gamepad2.right_stick_y);
                Arm2.openServo();
            }
            else if(!auto && !searchCone && !autoCone && !resetArm1 && !resetArm2 && !lbumper && !rbumper){
                Arm2.moveArm(0);
            }

            if(gamepad2.left_stick_y <= -.1 && !auto && !searchCone && !autoCone){
                Arm1.moveArm(-gamepad2.left_stick_y * 0.65);
            }
            else if(gamepad2.left_stick_y >= .1 && !auto && !searchCone && !autoCone){
                Arm1.moveArm(-gamepad2.left_stick_y * 0.65);
            }
            else if(!auto && !searchCone && !autoCone && !resetArm1 && !resetArm2 && !lbumper && !rbumper){
                Arm1.moveArm(0);
            }


            if(gamepad2.dpad_up){
                Arm1.forearmShort();
                armMid = true;
            }
            if(gamepad2.dpad_down){
                Arm1.resetArm();
                Arm2.resetArm();
                armModes();
            }
            if(gamepad2.dpad_right){
                Arm1.forearmMid();
                armMid = false;
            }


            if(gamepad2.left_bumper){
                lbumper = true;
                Arm2.closeServo();
            }
            if(lbumper && !auto && !searchCone && !autoCone){
                Arm2.moveArm(.6);
                if(Arm2.getArmPosition() > 230){
                    lbumper = false;
                    Arm2.moveArm(0);
                }
            }


            if(gamepad2.right_bumper){
                rbumper = true;
                Arm2.closeServo();
            }
            if(rbumper && !auto && !searchCone && !autoCone){
                if(Arm2.getArmPosition() < 405){
                    Arm2.moveArm(.75);
                }
                else{
                    Arm2.moveArm(.25);
                }
                if(Arm2.getArmPosition() > 430){
                    rbumper = false;
                    Arm2.moveArm(0);
                }
            }


            if(gamepad2.right_trigger > .5 && !armManualUp){
                armManualUp = true;
            }
            else if(gamepad2.right_trigger < .2 && armManualUp){
                if(heightManual > .85){
                    heightManual -= .025;
                }
                Arm1.forearmSpecDown(heightManual);
                armManualUp = false;
            }

            if(gamepad2.left_trigger > .5 && !armManualDown){
                armManualDown = true;
            }
            else if(gamepad2.left_trigger < .2 && armManualDown){
                if(heightManual < .95){
                    heightManual += .025;
                }
                Arm1.forearmSpecDown(heightManual);
                armManualDown = false;
            }
            /**
             * Autonomous Movements
             */

            /** Cycle High Pole **/
            if(auto && !autoCone && !searchCone){
                if(arm1OutFlag){
                    arm1OutFlag = false;
                    Arm1.forearmDownNoRotate();
                    Arm1.openClaw();
                    arm1Out = true;
                    senseConeTimer = timeyBoi.time();
                    rotate = true;
                }

                if(arm1Out){
                    Arm1.moveArm(.5);
                    if(Arm1.getArmPosition() > 15 && rotate){
                        Arm1.rotaterDown();
                        rotate = false;
                    }
                    if(Arm1.getArmPosition() > 160){
                        arm1Out = false;
                        Arm1.moveArm(0);
                    }
                }

                if(timeyBoi.time() - senseConeTimer > .25){
                    senseConeTimer = 420;
                    senseCone = true;
                }

                if(senseCone){
                    Arm1.moveArm(.15);
                    if(Arm1.clawSensor.getDistance(DistanceUnit.CM) < 5){
                        senseCone = false;
                        Arm1.moveArm(0);
                    }
                }

                if(grab){
                    Arm1.moveArm(.15);
                    if(Arm1.clawSensor.getDistance(DistanceUnit.CM) < 3){
                        senseCone = false;
                        arm1IsOut = true;
                        Arm1.moveArm(0);
                    }
                }

                if(arm1IsOut && grab){
                    Arm1.closeClaw();
                    arm1IsOut = false;
                    haveCone = true;
                    grab = false;
                }

                if(haveCone){
                    coneTimer = timeyBoi.time();
                    haveCone = false;
                }

                if(timeyBoi.time() - coneTimer > .3 && !arm1In){
                    arm1In = true;
                    Arm1.forearmUpNoRotate();
                }

                if(timeyBoi.time() - coneTimer > .5){
                    Arm1.rotaterUp();
                    coneTimer = 420;
                }

                if(arm1In){
                    Arm1.moveArm(-.7);
                    if(Arm1.getArmPosition() < 20){
                        arm1In = false;
                        arm1IsIn = true;
                        Arm1.moveArm(0);
                    }
                }

                if(arm1IsIn){
                    arm1IsIn = false;
                    arm1InTimerFlag = true;
                }

                if(arm1InTimerFlag){
                    arm1InTimer = timeyBoi.time();
                    arm1InTimerFlag = false;
                }

                if(timeyBoi.time() - arm1InTimer > .6){
                    Arm1.openClaw();
                }

                if(timeyBoi.time() - arm1InTimer > .65){
                    arm1OutFlag = true;
                }

                if(timeyBoi.time() - arm1InTimer > .7){
                    arm1InTimerFlag = false;
                    arm1InTimer = 420;
                    arm2Up = true;
                }

                if(arm2Up){
                    cycle++;
                    if(!arm2UpSlow){
                        Arm2.moveArm(1);
                    }
                    if(Arm2.getArmPosition() > 90){
                        Arm2.closeServo();
                    }
                    if(Arm2.getArmPosition() > 415 && !arm2UpSlow){
                        arm2UpSlow = true;
                        Arm2.moveArm(0.25);
                    }
                    if(Arm2.getArmPosition() > 455 && arm2UpSlow){
                        arm2IsUp = true;
                        arm2Up = false;
                        arm2UpSlow = false;
                        Arm2.moveArm(0);
                    }
                }

                if(arm2IsUp){
                    arm2Timer = timeyBoi.time();
                    arm2IsUp = false;
                }

                if(timeyBoi.time() - arm2Timer > .1 && !arm2Down) {
                    arm2Timer = 420;
                    arm2Down = true;
                    grabFlag = true;
                }

                if(arm2Down){
                    Arm2.moveArm(-1);
                    if(Arm2.getArmPosition() < 440){
                        Arm2.openServo();
                    }
                    if(Arm2.getArmPosition() < 300 && grabFlag){
                        grab = true;
                        grabFlag = false;

                    }
                    if(Arm2.getArmPosition() < 30){
                        arm2Down = false;
                        arm2IsDown = true;
                        Arm2.moveArm(0);
                        //Arm2.resetArm();
                    }
                }
            }

            /** Auto Pick Up Cone **/
            if(autoCone && !auto && !searchCone){
                if(arm1OutFlag){
                    arm1OutFlag = false;
                    Arm1.forearmDown();
                    Arm1.openClaw();
                    arm1OutTimer = timeyBoi.time();
                }

                if(timeyBoi.time() - arm1OutTimer > 1){
                    senseCone = true;
                    arm1OutTimer = 420;
                }

                if(senseCone){
                    Arm1.moveArm(.15);
                    if(Arm1.clawSensor.getDistance(DistanceUnit.CM) < 3){
                        senseCone = false;
                        arm1IsOut = true;
                        Arm1.moveArm(0);
                    }
                }

                if(arm1IsOut){
                    Arm1.closeClaw();
                    arm1IsOut = false;
                    haveCone = true;
                }

                if(haveCone){
                    coneTimer = timeyBoi.time();
                    haveCone = false;
                }

                if(timeyBoi.time() - coneTimer > .3 && !arm1In){
                    arm1In = true;
                    Arm1.forearmUpNoRotate();
                }

                if(timeyBoi.time() - coneTimer > .35){
                    Arm1.rotaterUp();
                    coneTimer = 420;
                }

                if(arm1In){
                    Arm1.moveArm(-.7);
                    if(Arm1.getArmPosition() < 5){
                        arm1In = false;
                        arm1IsIn = true;
                        Arm1.moveArm(0);
                        Arm1.resetArm();
                    }
                }

                if(arm1IsIn){
                    arm1IsIn = false;
                    arm1InTimerFlag = true;
                }

                if(arm1InTimerFlag){
                    arm1InTimer = timeyBoi.time();
                    arm1InTimerFlag = false;
                }

                if(timeyBoi.time() - arm1InTimer > .55){
                    Arm1.openClaw();
                    arm1InTimerFlag = false;
                    arm1InTimer = 420;
                    autoCone =  false;
                    resetAuto();
                }
            }


            /** Auto Search For Cone **/
            if(searchCone && !auto && !autoCone){
                Arm1.moveArm(.15);
                if(Arm1.clawSensor.getDistance(DistanceUnit.CM) < 3){
                    searchCone = false;
                    Arm1.moveArm(0);
                }
            }


            /** 5-stack autonomous **/
            if(autonomous && !searchCone && !auto && !autoCone){
                resetAuto();
                while(cycle < 6){
                    if(gamepad1.dpad_down && !autoFlag){
                        autoFlag = true;
                    }
                    if(!gamepad1.dpad_down && autoFlag){
                        autoFlag = false;
                        autonomous = false;
                        break;
                    }
                    if(arm1OutFlag){
                        arm1OutFlag = false;
                        if(cycle == 0){
                            Arm1.forearmSpecDown(.85);
                        }
                        else{
                            Arm1.forearmSpecDown(.85 + (0.025 * (cycle)));
                            Arm1.rotaterUp();
                        }
                        Arm1.openClaw();
                        arm1OutFlag = false;
                        arm1Out = true;
                        rotate = true;
                    }

                    if(arm1Out){
                        Arm1.moveArm(.35);
                        if(Arm1.getArmPosition() > 25 && rotate){
                            Arm1.rotaterDown();
                            rotate = false;
                        }
                        if(Arm1.getArmPosition() > 395){
                            arm1Out = false;
                            senseCone = true;
                            Arm1.moveArm(0);
                        }
                    }

                    if(senseCone){
                        Arm1.moveArm(.15);
                        if(Arm1.clawSensor.getDistance(DistanceUnit.CM) < 5){
                            senseCone = false;
                            Arm1.moveArm(0);
                        }
                    }

                    if(grab){
                        Arm1.moveArm(.15);
                        if(Arm1.clawSensor.getDistance(DistanceUnit.CM) < 3){
                            senseCone = false;
                            arm1IsOut = true;
                            Arm1.moveArm(0);
                        }
                    }

                    if(arm1IsOut && grab){
                        Arm1.closeClaw();
                        arm1IsOut = false;
                        haveCone = true;
                        grab = false;
                    }

                    if(haveCone){
                        coneTimer = timeyBoi.time();
                        haveCone = false;
                    }

                    if(timeyBoi.time() - coneTimer > .3 && !arm1In){
                        arm1In = true;
                        Arm1.forearmUpNoRotate();
                    }

                    if(timeyBoi.time() - coneTimer > .5){
                        Arm1.rotaterUp();
                        coneTimer = 420;
                    }

                    if(arm1In){
                        Arm1.moveArm(-.5);
                        if(Arm1.getArmPosition() < 5){
                            arm1In = false;
                            arm1IsIn = true;
                            Arm1.moveArm(0);
                            Arm1.resetArm();
                        }
                    }

                    if(arm1IsIn){
                        arm1IsIn = false;
                        arm1InTimerFlag = true;
                    }

                    if(arm1InTimerFlag){
                        arm1InTimer = timeyBoi.time();
                        arm1InTimerFlag = false;
                    }

                    if(timeyBoi.time() - arm1InTimer > .35){
                        Arm1.openClaw();
                    }

                    if(timeyBoi.time() - arm1InTimer > .45){
                        arm1InTimerFlag = false;
                        arm1InTimer = 420;
                        arm2Up = true;
                        cycleFlag = true;
                    }

                    if(arm2Up){
                        if(cycle < 5){
                            arm1OutFlag = true;
                        }
                        if(cycleFlag){
                            cycle++;
                            cycleFlag = false;
                        }
                        if(cycle == 5){
                            arm1OutFlag = false;
                        }
                        if(!arm2UpSlow){
                            Arm2.moveArm(.8);
                        }
                        if(Arm2.getArmPosition() > 60){
                            Arm2.closeServo();
                        }
                        if(Arm2.getArmPosition() > 415 && !arm2UpSlow){
                            arm2UpSlow = true;
                            Arm2.moveArm(0.25);
                        }
                        if(Arm2.getArmPosition() > 455 && arm2UpSlow){
                            arm2IsUp = true;
                            arm2Up = false;
                            arm2UpSlow = false;
                            Arm2.moveArm(0);
                        }
                    }

                    if(arm2IsUp){
                        arm2Timer = timeyBoi.time();
                        arm2IsUp = false;
                    }

                    if(timeyBoi.time() - arm2Timer > .1 && !arm2Down) {
                        if(cycle == 5){
                            Arm1.forearmDown();
                        }
                        arm2Timer = 420;
                        arm2Down = true;
                        grabFlag = true;
                        Arm2.openServo();
                    }

                    if(arm2Down){
                        Arm2.moveArm(-1);
                        if(Arm2.getArmPosition() < 300 && grabFlag){
                            if(cycle != 5){
                                grab = true;
                            }
                            grabFlag = false;

                        }
                        if(Arm2.getArmPosition() < 5){
                            arm2Down = false;
                            arm2IsDown = true;
                            Arm2.moveArm(0);
                            if(cycle == 5 ){
                                break;
                            }
                        }
                    }
                }
                resetAuto();
                Arm1.forearmUp();
                cycleFlag = false;
                autonomous = false;
            }

            totalCurrent = (DriveTrain.leftFront.getCurrent(CurrentUnit.MILLIAMPS) + DriveTrain.leftBack.getCurrent(CurrentUnit.MILLIAMPS) + DriveTrain.rightFront.getCurrent(CurrentUnit.MILLIAMPS) + DriveTrain.rightBack.getCurrent(CurrentUnit.MILLIAMPS) + Arm1.arm1Left.getCurrent(CurrentUnit.MILLIAMPS) + Arm1.arm1Right.getCurrent(CurrentUnit.MILLIAMPS) + Arm2.arm2Left.getCurrent(CurrentUnit.MILLIAMPS) + Arm2.arm2Right.getCurrent(CurrentUnit.MILLIAMPS));
            if(totalCurrent > 20000 && !totalFlag){
                totalFlag = true;
                totalTimer = timeyBoi.time();
            }
            if(totalFlag){
                if(totalCurrent < 20000){
                    totalFlag = false;
                }
                else if(timeyBoi.time() - totalTimer > 1){

                }
            }

            //try.addData("Arm 1 pos: ", Arm1.getArmPosition());
            // telemetry.addData("Cycle: ", cycle);
            //telemetry.addData("Dist: ", Arm1.clawSensor.getDistance(DistanceUnit.CM));
            //telemetry.addData("Total current: ", totalCurrent);
            //telemetry.update();
        }
    }

    public static void resetAuto(){
        arm1Out = false;
        arm1OutFlag = true;
        senseConeTimer = 420;
        senseCone = false;
        arm1IsOut = false;
        haveCone = false;
        coneTimer = 420;
        arm1In = false;
        arm1IsIn = false;
        arm1InTimerFlag = false;
        arm1InTimer = 420;
        arm2Up = false;
        arm2UpSlow = false;
        arm2IsUp = false;
        arm2Timer = 420;
        arm2Down = false;
        arm2IsDown = false;
        grab = true;
        grabFlag = false;
        cycle = 0;
    }

    public static void armModes(){
        Arm1.arm1Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Arm1.arm1Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Arm2.arm2Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Arm2.arm2Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
