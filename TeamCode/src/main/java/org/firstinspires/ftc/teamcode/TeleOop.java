package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    public static boolean cycleFlag = false;
    public static double cycle = 0;

    public static boolean rbumper = false;
    public static boolean rbumperSlow = false;
    public static boolean lbumper = false;
    public static boolean resetArm1 = false;
    public static boolean resetArm2 = false;

    public static boolean fieldCentric = false;


    public void runOpMode() throws InterruptedException {
        Arm1.initArm1(hardwareMap);
        Arm2.initArm2(hardwareMap);
        DriveTrain.initDriveTrain(hardwareMap);

        waitForStart();

        timeyBoi.reset();

        while(opModeIsActive()){

            /**
             * Gamepad1 Controls
             */

            if(!fieldCentric){
                DriveTrain.mecanumDrive(gamepad1.left_stick_y * 0.55, -gamepad1.left_stick_x * 0.55, gamepad1.right_stick_x * 0.35);
            }
            else{
                DriveTrain.cartesianDrive((-gamepad1.left_stick_x / 1.75), (gamepad1.left_stick_y / 1.75), (gamepad1.right_stick_x / 2.5), Math.PI);
            }

            if(gamepad1.dpad_right && !fieldCentric){
                DriveTrain.resetGyro();
                fieldCentric = true;
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
                Arm1.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE);
            }
            if(gamepad1.right_bumper){
                Arm1.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
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


            /**
             * Gamepad2 Controls
             */

            if(gamepad2.a){
                Arm1.forearmDown();
            }
            if(gamepad2.b){
                Arm1.closeClaw();
            }
            if(gamepad2.x){
                Arm1.openClaw();
            }
            if(gamepad2.y){
                Arm1.forearmUp();
            }


            if(gamepad2.right_stick_y <= -.1 && !auto && !searchCone && !autoCone){
                Arm2.moveArm(-gamepad2.right_stick_y * 0.65);
            }
            else if(gamepad2.right_stick_y >= .1 && !auto && !searchCone && !autoCone){
                Arm2.moveArm(-gamepad2.right_stick_y * 0.65);
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
            }
            if(gamepad2.dpad_down){
                Arm1.resetArm();
                Arm2.resetArm();
                armModes();
            }


            if(gamepad2.left_bumper){
                lbumper = true;
            }
            if(lbumper && !auto && !searchCone && !autoCone){
                Arm2.moveArm(.85);
                if(Arm2.getArmPosition() > 265){
                    lbumper = false;
                    Arm2.moveArm(0);
                }
            }


            if(gamepad2.right_bumper){
                rbumper = true;
            }
            if(rbumper && !auto && !searchCone && !autoCone){
                Arm2.moveArm(.85);
                if(Arm2.getArmPosition() > 480){
                    rbumper = false;
                    Arm2.moveArm(0);
                }
            }


            if(gamepad2.right_trigger > .5){
                searchCone = true;
            }
            else{
                searchCone = false;
            }
            /**
             * Autonomous Movements
             */

            /** Cycle High Pole **/
            if(auto && !autoCone && !searchCone){
                if(arm1OutFlag){
                    arm1OutFlag = false;
                    Arm1.forearmDown();
                    Arm1.openClaw();
                    arm1OutFlag = false;
                    arm1Out = true;
                    senseConeTimer = timeyBoi.time();
                }

                if(arm1Out){
                    Arm1.moveArm(.5);
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
                }

                if(timeyBoi.time() - arm1InTimer > .65){
                    arm1InTimerFlag = false;
                    arm1InTimer = 420;
                    arm2Up = true;
                }

                if(arm2Up){
                    arm1OutFlag = true;
                    cycle++;
                    Arm2.moveArm(.8);
                    if(Arm2.getArmPosition() > 480){
                        arm2IsUp = true;
                        arm2Up = false;
                        Arm2.moveArm(0);
                    }
                }

                if(arm2IsUp){
                    arm2Timer = timeyBoi.time();
                    arm2IsUp = false;
                }

                if(timeyBoi.time() - arm2Timer > .05 && !arm2Down){
                    arm2Timer = 420;
                    arm2IsUp = false;
                    arm2Down = true;
                    grabFlag = true;
                }

                if(arm2Down){
                    Arm2.moveArm(-1);
                    if(Arm2.getArmPosition() < 300 && grabFlag){
                        grab = true;
                        grabFlag = false;

                    }
                    if(Arm2.getArmPosition() < 5){
                        arm2Down = false;
                        arm2IsDown = true;
                        Arm2.moveArm(0);
                        Arm2.resetArm();
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
                        }
                        Arm1.openClaw();
                        arm1OutFlag = false;
                        arm1Out = true;
                    }

                    if(arm1Out){
                        Arm1.moveArm(.5);
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

                    if(timeyBoi.time() - arm1InTimer > .45){
                        Arm1.openClaw();
                    }

                    if(timeyBoi.time() - arm1InTimer > .55){
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
                        Arm2.moveArm(.8);
                        if(Arm2.getArmPosition() > 480){
                            arm2IsUp = true;
                            arm2Up = false;
                            Arm2.moveArm(0);
                        }
                    }

                    if(arm2IsUp){
                        arm2Timer = timeyBoi.time();
                        arm2IsUp = false;
                    }

                    if(timeyBoi.time() - arm2Timer > .05 && !arm2Down) {
                        if(cycle == 5){
                            Arm1.forearmDown();
                        }
                        arm2Timer = 420;
                        arm2IsUp = false;
                        arm2Down = true;
                        grabFlag = true;
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
                            Arm2.resetArm();
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

            telemetry.addData("Arm 1 pos: ", Arm1.getArmPosition());
            telemetry.addData("Cycle: ", cycle);
            telemetry.update();
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
