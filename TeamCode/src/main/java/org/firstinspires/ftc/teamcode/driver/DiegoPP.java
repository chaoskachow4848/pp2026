
package org.firstinspires.ftc.teamcode.driver;

//import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

import static org.firstinspires.ftc.teamcode.driver.PPDrive.D;
import static org.firstinspires.ftc.teamcode.driver.PPDrive.F;
import static org.firstinspires.ftc.teamcode.driver.PPDrive.I;
import static org.firstinspires.ftc.teamcode.driver.PPDrive.P;
import static org.firstinspires.ftc.teamcode.driver.PPDrive.aimerMax;
import static org.firstinspires.ftc.teamcode.driver.PPDrive.aimerMid;
import static org.firstinspires.ftc.teamcode.driver.PPDrive.aimerMin;
import static org.firstinspires.ftc.teamcode.driver.PPDrive.deflectorMiddle;
import static org.firstinspires.ftc.teamcode.driver.PPDrive.intakeFast;
import static org.firstinspires.ftc.teamcode.driver.PPDrive.leftFeederDown;
import static org.firstinspires.ftc.teamcode.driver.PPDrive.leftFeederUp;
import static org.firstinspires.ftc.teamcode.driver.PPDrive.rightFeederDown;
import static org.firstinspires.ftc.teamcode.driver.PPDrive.rightFeederUp;
import static org.firstinspires.ftc.teamcode.driver.PPDrive.shooterMax;
import static org.firstinspires.ftc.teamcode.driver.PPDrive.shooterMid;
import static org.firstinspires.ftc.teamcode.driver.PPDrive.shooterMin;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardware.KachowHardware;
import org.firstinspires.ftc.teamcode.hardware.KachowHardware.state;
import org.firstinspires.ftc.teamcode.hardware.Vector2d;
import org.firstinspires.ftc.teamcode.hardware.button;
import org.firstinspires.ftc.teamcode.hardware.kaze;

@Configurable
@TeleOp(name="DiegoPP", group="4848")
//@Disabled
public class DiegoPP extends LinearOpMode {

    public static double distance;
    public static Vector2d target;
    public static double shooterVelocity;
    public static double shooterFar = shooterMax;
    public static double shooterClose;
    public static double aimerPose;
    public static double launchTime;
    public boolean manual = false;
    public boolean doubleLaunch = false;
    public boolean powerMode = false;
    boolean green = false;
    boolean purple1 = false;
    boolean purple2 = false;
    public PathChain park;






    button.ButtonReader gamePad1 = new button.ButtonReader();
    button.ButtonReader gamePad2 = new button.ButtonReader();
    /* Declare OpMode members. */
    KachowHardware robot = new KachowHardware();
    state State = state.driving;

    ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        gamepad2.runLedEffect(robot.redled);
        gamepad1.runLedEffect(robot.blueled);
        telemetry.update();
        robot.pattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
        robot.rightFeeder.setPosition(rightFeederDown);
        robot.leftFeeder.setPosition(leftFeederDown);
        robot.deflector.setPosition(deflectorMiddle);
        if(kaze.robotPose == null){
            kaze.init(new Pose(72,72,0));//kaze init before robotinit
        } else {
            robot.kachow.drive.setPose(kaze.robotPose);
        }
        waitForStart();
        robot.runtime.reset();
        runtime.reset();
        if(kaze.target != null){
            target = kaze.target;
        } else {
            target = new Vector2d(0,144);
        }
        robot.aimer.setPosition(.63);







        while (opModeIsActive()) {
            update();




            robot.kachow.drive.updatePose();
            telemetry.addLine("x: " + robot.kachow.drive.getPose().getX());
            telemetry.addLine("y: " + robot.kachow.drive.getPose().getY());
            telemetry.addLine("heading: " + Math.toDegrees(robot.kachow.drive.getPose().getHeading()));
            telemetry.addLine("aimer: " + aimerPose);
            Vector2d botPoint = new Vector2d(robot.kachow.drive.getPose().getX(), robot.kachow.drive.getPose().getY());
            robot.kachow.bot_to_target = Math.sqrt(Math.abs((botPoint.x-target.x)*(botPoint.x-target.x) + (botPoint.y-target.y)*(botPoint.y-target.y)));
            aimerPose = ((robot.kachow.bot_to_target)/210);
            if (aimerPose>aimerMax){
                aimerPose = aimerMid;
            }else if (aimerPose<aimerMin){
                aimerPose = aimerMin;
            }
            shooterVelocity = 1380;//1600*((robot.kachow.bot_to_target)/100);
            if (shooterVelocity>shooterMid){
                shooterVelocity = shooterMid;
            }else if (shooterVelocity<shooterMin){
                shooterVelocity = shooterMin;
            }
            if(botPoint.y <= 60){
                aimerPose = aimerMax;
            } else {
                aimerPose = .46;
            }
            if(botPoint.y <= 60){
                shooterVelocity = shooterMax;
            }



            switch (State){
                case driving:
                    if (gamePad2.Dpad_Left.isDown()){
                        robot.leftFeeder.setPosition(leftFeederUp);
                    } else if (gamePad2.Dpad_Right.isDown()){
                        robot.rightFeeder.setPosition(rightFeederUp);
                    } else {
                        //robot.deflector.setPosition(1);
                        robot.leftFeeder.setPosition(leftFeederDown);
                        robot.rightFeeder.setPosition(rightFeederDown);
                    }
                    robot.spinnerLeft.setVelocity(0);
                    robot.spinnerRight.setVelocity(0);
                    if (gamePad1.Right_Trigger.isDown() || gamePad2.Right_Trigger.isDown()){
                        robot.intake.setPower(intakeFast);
                    } else {
                        robot.intake.setPower(0);
                    }

                    if (gamePad1.Left_Trigger.isDown() || gamePad2.Left_Trigger.isDown()){
                        robot.intake.setPower(-intakeFast);
                    }
                    robot.kachow.robotCentric(opModeIsActive(), gamepad1, gamepad2, robot);
                    if(!manual){
                        robot.aimer.setPosition(aimerPose);
                    }

                    if(gamePad1.Right_Bumper.wasJustPressed()){
                        //robot.deflector.setPosition(1);
                        changeStateTo(state.aimbot);
                        break;
                    }
                    if(gamePad1.Right_Trigger.isDown()){
                        changeStateTo(state.intaking);
                        break;
                    }
                    if(gamePad1.Left_Trigger.isDown()){
                        changeStateTo(state.intaking);
                        break;
                    }
                    if(gamepad1.options && gamepad1.share){
                        robot.drive.activateAllPIDFs();
                        if(!kaze.IsBlue){
                            park = robot.drive.pathBuilder()
                                    .addPath(new BezierLine(robot.drive.getPose(), new Pose(40.5, 30.7)))
                                    .setLinearHeadingInterpolation(robot.drive.getHeading(), Math.toRadians(180))
                                    .build();
                        } else {
                            park = robot.drive.pathBuilder()
                                    .addPath(new BezierLine(robot.drive.getPose(), new Pose(144-40.5, 30.7)))
                                    .setLinearHeadingInterpolation(robot.drive.getHeading(), Math.toRadians(180))
                                    .build();
                        }

                        changeStateTo(state.park);
                        robot.drive.followPath(park);
                    }
                    break;

                case intaking:
                    if (gamePad2.Dpad_Left.isDown()){
                        robot.leftFeeder.setPosition(leftFeederUp);
                    } else if (gamePad2.Dpad_Right.isDown()){
                        robot.rightFeeder.setPosition(rightFeederUp);
                    } else {
                        //robot.deflector.setPosition(1);
                        robot.leftFeeder.setPosition(leftFeederDown);
                        robot.rightFeeder.setPosition(rightFeederDown);
                    }
                    if(robot.stateChanged){

                        //robot.deflector.setPosition(1);
                        robot.spinnerLeft.setPower(.6);
                        robot.spinnerRight.setPower(.6);
                    }
                    robot.kachow.robotCentric(1 ,opModeIsActive(), gamepad1, gamepad2, robot, false);
                    if(!manual){
                        robot.aimer.setPosition(aimerPose);
                    }                    if (gamePad1.Right_Trigger.isDown() || gamePad2.Right_Trigger.isDown()){
                        robot.intake.setPower(intakeFast);
                    } else {
                        robot.intake.setPower(0);
                    }

                    if (gamePad1.Left_Trigger.isDown() || gamePad2.Left_Trigger.isDown()){
                        robot.intake.setPower(-intakeFast);
                    }


                    if(gamePad1.Left_Bumper.isDown()){
                        robot.spinnerLeft.setVelocity(shooterVelocity);
                        robot.spinnerRight.setVelocity(shooterVelocity);
                    } else {
                        robot.spinnerLeft.setPower(.6);
                        robot.spinnerRight.setPower(.6);
                    }


                    telemetry.addData("Intake: ", robot.intake.getVelocity());
                    telemetry.addData("IntakePower: ", robot.intake.getPower());
                    //robot.deflector.setPosition(1);
                    if(gamePad1.Right_Bumper.wasJustPressed()){
                        robot.intake.setPower(0);
                        ////robot.deflector.setPosition(1);
                        //robot.deflector.setPosition(1);
                        changeStateTo(state.aimbot);
                        break;
                    }
                    if(gamepad1.options && gamepad1.share){
                        robot.drive.activateAllPIDFs();
                        if(!kaze.IsBlue){
                            park = robot.drive.pathBuilder()
                                    .addPath(new BezierLine(robot.drive.getPose(), new Pose(40.5, 30.7)))
                                    .setLinearHeadingInterpolation(robot.drive.getHeading(), Math.toRadians(180))
                                    .build();
                        } else {
                            park = robot.drive.pathBuilder()
                                    .addPath(new BezierLine(robot.drive.getPose(), new Pose(144-40.5, 30.7)))
                                    .setLinearHeadingInterpolation(robot.drive.getHeading(), Math.toRadians(180))
                                    .build();
                        }
                        changeStateTo(state.park);
                        robot.drive.followPath(park);
                    }
                    break;

                case aimbot:
                    if(robot.stateChanged){
                        launchTime = -5;
                    }
                    //robot.deflector.setPosition(1);
                    robot.kachow.aimbot(target, gamepad1, gamepad2, robot, .18);
                    if(gamepad1.right_stick_x != 0){
                        target = new Vector2d(target.x+gamepad1.right_stick_x, target.y);
                    }
                    if(gamepad2.right_stick_x != 0){
                        target = new Vector2d(target.x+gamepad2.right_stick_x, target.y);
                    }
                   /* if (gamePad2.triangle.wasJustPressed()){
                        shooterVelocity = shooterVelocity+100;
                        robot.spinner.setVelocity(shooterVelocity);
                    }
                    if (gamePad2.x.wasJustPressed()){
                        shooterVelocity = shooterVelocity-100;
                        robot.spinner.setVelocity(shooterVelocity);
                    }
                    */

                    if(!doubleLaunch) {
                        //robot.spinner.setPower(robot.spinner.getPower() + (shooterVelocity-robot.spinner.getVelocity()) * .00001);
                        //powerMode = true;
                        //robot.deflector.setPosition(1);
                        robot.spinnerLeft.setVelocity(shooterVelocity);//+140
                        robot.spinnerRight.setVelocity(shooterVelocity);//+140
                    } else {
                        robot.spinnerLeft.setVelocity(shooterVelocity);
                        robot.spinnerRight.setVelocity(shooterVelocity);
                        //robot.intake.setPower(.5);
                        if (((robot.stateTime.seconds()-launchTime) >= .8) && ((robot.stateTime.seconds()-launchTime) <= 1)){
                            //robot.deflector.setPosition(deflectorMiddle);
                        }
                    }
                    robot.spinnerLeft.setVelocity(shooterVelocity);//+140
                    robot.spinnerRight.setVelocity(shooterVelocity);//+140
/*

                    if(shooterVelocity<0){
                        shooterVelocity = 0;
                    }

                    difference = Math.abs(robot.spinner.getVelocity()-shooterVelocity);

                    if(robot.spinner.getVelocity()<shooterVelocity-21)
                    {
                        if (robot.spinner.getPower() >= 1){
                            robot.spinner.setPower(1);
                        } else {
                            robot.spinner.setPower(robot.spinner.getPower()+(difference*.000001));
                        }
                    }/*
                    if(robot.spinner.getVelocity()>shooterVelocity+21)
                    {
                        //robot.spinner.setPower(0);
                        if(difference>=200){
                            robot.spinner.setPower(robot.spinner.getPower()-(difference*.00001));
                        }else{
                            robot.spinner.setPower(robot.spinner.getPower()-(difference*.000005));
                        }
                    }
                    if(robot.spinner.getPower() < 0){
                        robot.spinner.setPower(0);
                    }*/
                    if((robot.spinnerLeft.getVelocity() >= (shooterVelocity-20)) && (robot.spinnerLeft.getVelocity() <= (shooterVelocity+20))){

                            if (gamePad1.Dpad_Left.isDown() || gamePad1.Dpad_Up.isDown()){
                                //robot.deflector.setPosition(deflectorMiddle);
                                robot.leftFeeder.setPosition(leftFeederUp);
                                ////robot.deflector.setPosition(1);
                                launchTime = robot.stateTime.seconds();
                            }


                    }


                    if((robot.spinnerRight.getVelocity() >= (shooterVelocity-20)) && (robot.spinnerRight.getVelocity() <= (shooterVelocity+20))){
                        if (gamePad1.Dpad_Right.isDown() || gamePad1.Dpad_Up.isDown()){
                            robot.rightFeeder.setPosition(rightFeederUp);
                            //robot.deflector.setPosition(deflectorLeftIn);
                            launchTime = robot.stateTime.seconds();
                        }

                    }
                    if((robot.spinnerRight.getVelocity() >= (shooterVelocity-20)) && (robot.spinnerRight.getVelocity() <= (shooterVelocity+20))) {
                        if ((robot.spinnerLeft.getVelocity() >= (shooterVelocity - 20)) && (robot.spinnerLeft.getVelocity() <= (shooterVelocity + 20))) {
                            gamepad1.rumble(50);
                        }
                    }

                /*    if(!doubleLaunch){
                        if((robot.spinner.getVelocity() >= (shooterVelocity+100)) && (robot.spinner.getVelocity() <= (shooterVelocity+180))){
                            gamepad2.rumble(50);

                            if(robot.stateTime.seconds() - launchTime >= 1){
                                if(gamePad2.triangle.isDown()){
                                    doubleLaunch = true;
                                }

                                if (gamePad2.Dpad_Right.isDown() || (gamePad2.Right_Trigger.isDown() || gamePad2.Left_Trigger.isDown())){
                                    robot.rightFeeder.setPosition(rightFeederUp);
                                    //robot.deflector.setPosition(deflectorLeftIn);
                                    launchTime = robot.stateTime.seconds();
                                }

                                if (gamePad2.Dpad_Left.isDown() || (gamePad2.Right_Trigger.isDown() || gamePad2.Left_Trigger.isDown())){
                                    //robot.deflector.setPosition(deflectorMiddle);
                                    robot.leftFeeder.setPosition(leftFeederUp);
                                    ////robot.deflector.setPosition(1);
                                    launchTime = robot.stateTime.seconds();
                                }

                            }
                        }

                    }

                 */







                    if(((robot.leftFeeder.getPosition() > leftFeederUp-.05) || ((robot.rightFeeder.getPosition() > rightFeederUp-.05))) && (((robot.stateTime.seconds()-launchTime) >= .25) && ((robot.stateTime.seconds()-launchTime) <= .4))){
                        robot.rightFeeder.setPosition(rightFeederDown);
                        //robot.deflector.setPosition(deflectorMiddle);
                        robot.leftFeeder.setPosition(leftFeederDown);
                        robot.intake.setPower(-.3);
                    } else if (((robot.stateTime.seconds()-launchTime) > .4) && (robot.stateTime.seconds()-launchTime) < .8){
                        robot.intake.setPower(.5);
                        robot.leftFeeder.setPosition(leftFeederDown);
                        robot.rightFeeder.setPosition(rightFeederDown);
                        // //robot.deflector.setPosition(1);
                    }else if (((robot.stateTime.seconds()-launchTime) >= .8) && ((robot.stateTime.seconds()-launchTime) <= 1)){
                        if (gamePad1.Right_Trigger.isDown() || gamePad2.Right_Trigger.isDown()){
                            robot.intake.setPower(intakeFast);
                        } else {
                            robot.intake.setPower(0);
                        }

                        if (gamePad1.Left_Trigger.isDown() || gamePad2.Left_Trigger.isDown()){
                            robot.intake.setPower(-intakeFast);
                        }                    }

                    if(gamePad1.Dpad_Up.wasJustPressed()){
                        robot.aimer.setPosition(robot.aimer.getPosition()+.01);
                        manual = true;
                    }
                    if(gamePad1.Dpad_Down.wasJustPressed()){
                        robot.aimer.setPosition(robot.aimer.getPosition()-.01);
                        manual = true;
                    } if (!manual){
                    robot.aimer.setPosition(aimerPose);
                }



                    robot.spinnerLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P,I, D, F));
                    robot.spinnerRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P,I, D, F));

                    telemetry.addData("spinnerLeft: ", robot.spinnerLeft.getVelocity());
                    telemetry.addData("spinnerPowerLeft: ", robot.spinnerLeft.getPower());


                    telemetry.addData("spinnerRight: ", robot.spinnerRight.getVelocity());
                    telemetry.addData("spinnerPowerRight: ", robot.spinnerRight.getPower());
                    telemetry.addData("TargetVelocity: ", shooterVelocity);
                    telemetry.addData("P: ", P);
                    telemetry.addData("I: ", I);
                    telemetry.addData("D: ", D);
                    telemetry.addData("F: ", F);
                    if(gamePad1.Right_Bumper.wasJustReleased()){
                        robot.rightFeeder.setPosition(rightFeederDown);
                        robot.leftFeeder.setPosition(leftFeederDown);
                        changeStateTo(state.driving);
                        manual = false;
                        powerMode = false;
                        doubleLaunch = false;
                        launchTime = 0;
                    }
                    if(gamePad1.triangle.isDown()){
                        robot.rightFeeder.setPosition(rightFeederDown);
                        robot.leftFeeder.setPosition(leftFeederDown);
                        changeStateTo(state.launching);
                        manual = false;
                        powerMode = false;
                        doubleLaunch = false;
                        launchTime = 0;
                    }
                    break;

                case launching:
                    if(robot.stateChanged){
                        launchTime = -5;
                    }
                    //robot.deflector.setPosition(1);
                    robot.kachow.aimbot(target, gamepad1, gamepad2, robot, .18);
                if(gamepad1.right_stick_x != 0){
                        target = new Vector2d(target.x+gamepad1.right_stick_x, target.y);
                }
                  if(launch(kaze.pattern, (int) shooterVelocity)){
                      changeStateTo(state.driving);
                      green = false;
                      purple1 = false;
                      purple2 = false;
                  } else {

                      if (!doubleLaunch) {
                          //robot.spinner.setPower(robot.spinner.getPower() + (shooterVelocity-robot.spinner.getVelocity()) * .00001);
                          //powerMode = true;
                          //robot.deflector.setPosition(1);
                          robot.spinnerLeft.setVelocity(shooterVelocity);//+140
                          robot.spinnerRight.setVelocity(shooterVelocity);//+140
                      } else {
                          robot.spinnerLeft.setVelocity(shooterVelocity);
                          robot.spinnerRight.setVelocity(shooterVelocity);
                          //robot.intake.setPower(.5);
                          if (((robot.stateTime.seconds() - launchTime) >= .8) && ((robot.stateTime.seconds() - launchTime) <= 1)) {
                              //robot.deflector.setPosition(deflectorMiddle);
                          }
                      }

                      if (gamePad1.Dpad_Up.wasJustPressed()) {
                          robot.aimer.setPosition(robot.aimer.getPosition() + .01);
                          manual = true;
                      }
                      if (gamePad1.Dpad_Down.wasJustPressed()) {
                          robot.aimer.setPosition(robot.aimer.getPosition() - .01);
                          manual = true;
                      }
                      if (!manual) {
                          robot.aimer.setPosition(aimerPose);
                      }


                      robot.spinnerLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));
                      robot.spinnerRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));

                      telemetry.addData("spinner: ", robot.spinnerLeft.getVelocity());
                      telemetry.addData("spinnerPower: ", robot.spinnerLeft.getPower());
                      telemetry.addData("TargetVelocity: ", shooterVelocity);
                      telemetry.addData("P: ", P);
                      telemetry.addData("I: ", I);
                      telemetry.addData("D: ", D);
                      telemetry.addData("F: ", F);

                  }


                    break;


                case park:
                    if(robot.stateChanged){
                        robot.rightFeeder.setPosition(rightFeederDown);
                        robot.leftFeeder.setPosition(leftFeederDown);
                        robot.spinnerLeft.setPower(0);
                        robot.spinnerRight.setPower(0);
                        robot.intake.setPower(0);
                    }else {
                        robot.rightFeeder.setPosition(rightFeederDown);
                        robot.leftFeeder.setPosition(leftFeederDown);
                        robot.spinnerLeft.setPower(0);
                        robot.spinnerRight.setPower(0);
                        robot.intake.setPower(0);
                    }



                    if(gamepad1.touchpad || gamepad2.touchpad){
                        robot.drive.deactivateAllPIDFs();
                        robot.drive.pausePathFollowing();
                        robot.drive.breakFollowing();
                        changeStateTo(state.driving);
                        manual = false;
                        powerMode = false;
                        launchTime = 0;
                    }
            }

            telemetry.addData("Aimer: ", robot.aimer.getPosition());



            if (gamePad2.Dpad_Left.isDown()){
                robot.leftFeeder.setPosition(leftFeederUp);
            }
            if (gamePad2.Dpad_Right.isDown()){
                robot.rightFeeder.setPosition(rightFeederUp);
            }


            if(gamePad2.Left_Bumper.isDown() || gamePad2.Right_Bumper.isDown()){
                robot.deflector.setPosition(deflectorMiddle);
            } else {
                robot.deflector.setPosition(1);
            }

            if(gamePad2.Dpad_Up.wasJustPressed()){
                robot.aimer.setPosition(robot.aimer.getPosition()+.01);
                manual = true;
            }
            if(gamePad2.Dpad_Down.wasJustPressed()){
                robot.aimer.setPosition(robot.aimer.getPosition()-.01);
                manual = true;
            } if (!manual){
                robot.aimer.setPosition(aimerPose);
            }






        }
    }

    public void robot_robotCentric(double DriveSpeed, boolean slow) {
        double FrontLeft;
        double FrontRight;
        double BackLeft;
        double BackRight;
        telemetry.addLine(String.valueOf(distance));
        telemetry.addData(">", runtime.seconds());
        telemetry.update();
        if(runtime.seconds() > 84.8 && runtime.seconds() < 85.2){
            gamepad1.rumble(5000);
            gamepad2.rumble(5000);
        }
        if(runtime.seconds() > 109 && runtime.seconds() < 110){
            gamepad1.rumble(10000);
            gamepad2.rumble(10000);
        }
        //StrafeRight
        if (opModeIsActive()) {
            double x = -gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;
            double turn = -gamepad1.right_stick_x;
            double theta = Math.atan2(y, x);
            double power = Math.hypot(x, y);

            double sin = Math.sin(theta - Math.PI/4);
            double cos = Math.cos(theta - Math.PI/4);
            double max = Math.max(Math.abs(sin), Math.abs(cos));

            FrontLeft = power * cos/max + turn;
            FrontRight = power * sin/max - turn;
            BackLeft = power * sin/max + turn;
            BackRight = power * cos/max - turn;

            if ((power + Math.abs(turn)) > 1){
                FrontLeft /= power + turn;
                FrontRight /= power + turn;
                BackLeft /= power + turn;
                BackRight /= power + turn;
            }

            robot.backleft.setPower(Range.clip(BackLeft, -DriveSpeed, DriveSpeed));
            robot.backright.setPower(Range.clip(BackRight, -DriveSpeed, DriveSpeed));
            robot.frontleft.setPower(Range.clip(FrontLeft, -DriveSpeed, DriveSpeed));
            robot.frontright.setPower(Range.clip(FrontRight, -DriveSpeed, DriveSpeed));
        }

    }
    public void update(){ //place once on start of loop
        robot.drive.update();
        gamePad2.update(gamepad2, robot);
        robot.stateUpdate(State);
        gamePad1.update(gamepad1, robot);
        if(gamepad2.share && gamepad2.options){
            robot.kachow.drive.setPose(new Pose(72, 72, 0));
        }
        kaze.update(robot.kachow.drive);
        telemetry.addData("state: ", State);
        telemetry.update();
        kaze.drawCurrentAndHistory(robot.kachow.drive);
    }
    public void changeStateTo(state tostate){
        State = tostate;
        robot.stateUpdate(State);
    }//hellooooo
    public boolean launch(String pattern, int velocity) {
        //left is green
        //right is purples

        telemetry.addData("launchtime", launchTime);
        if(purple1){
            telemetry.addLine("purple1");
        }
        if(purple2){
            telemetry.addLine("purple2");
        }
        if(green){
            telemetry.addLine("green");
        }
        robot.spinnerLeft.setVelocity(velocity);
        robot.spinnerRight.setVelocity(velocity);
        switch (pattern) {
            case "PPG":
                    if (robot.stateTime.seconds() - launchTime >= 1) {
                        //if not shooting
                        if ((purple1 && purple2 && !green) && ((robot.spinnerLeft.getVelocity() >= (velocity - 20)) && (robot.spinnerLeft.getVelocity() <= (velocity + 20)))) {
                            //if ready to shoot
                            //shoot third
                            robot.leftFeeder.setPosition(leftFeederUp);
                            robot.rightFeeder.setPosition(rightFeederUp);
                            //robot.deflector.setPosition(deflectorMiddle);
                            launchTime = robot.stateTime.seconds();
                            green = true;
                        } else if ((purple1 && !purple2 && !green) && ((robot.spinnerRight.getVelocity() >= (velocity - 20)) && (robot.spinnerRight.getVelocity() <= (velocity + 20)))) {
                            //if ready to shoot
                            //shoot second
                            robot.rightFeeder.setPosition(rightFeederUp);
                            //robot.deflector.setPosition(1);
                            launchTime = robot.stateTime.seconds();
                            purple2 = true;
                        } else if ((!purple1 && !purple2 && !green) && ((robot.spinnerRight.getVelocity() >= (velocity - 20)) && (robot.spinnerRight.getVelocity() <= (velocity + 20)))) {
                            //if ready to shoot
                            //shoot first
                            robot.rightFeeder.setPosition(rightFeederUp);
                            //robot.deflector.setPosition(1);
                            launchTime = robot.stateTime.seconds();
                            purple1 = true;
                        }
                    }

                if(((robot.leftFeeder.getPosition() > leftFeederUp-.05) || ((robot.rightFeeder.getPosition() > rightFeederUp-.05))) && (((robot.stateTime.seconds()-launchTime) >= .25) && ((robot.stateTime.seconds()-launchTime) <= .4))){
                    robot.leftFeeder.setPosition(leftFeederDown);
                    robot.rightFeeder.setPosition(rightFeederDown);
                    //robot.deflector.setPosition(deflectorMiddle);
                    robot.intake.setPower(-.45);
                } else if (((robot.stateTime.seconds()-launchTime) >= .4) && (robot.stateTime.seconds()-launchTime) < 1){
                    robot.intake.setPower(1);
                    //robot.deflector.setPosition(1);
                }else if (((robot.stateTime.seconds()-launchTime) >= 1)){
                    if (gamePad1.Right_Trigger.isDown() || gamePad2.Right_Trigger.isDown()){
                        robot.intake.setPower(intakeFast);
                    } else {
                        robot.intake.setPower(0);
                    }

                    if (gamePad1.Left_Trigger.isDown() || gamePad2.Left_Trigger.isDown()){
                        robot.intake.setPower(-intakeFast);
                    }
                    return (purple1 && purple2 && green);
                }

                break;
            case "PGP":
                    if (robot.stateTime.seconds() - launchTime >= 1) {
                        //if not shooting
                        if ((purple1 && green && !purple2) && ((robot.spinnerRight.getVelocity() >= (velocity - 20)) && (robot.spinnerRight.getVelocity() <= (velocity + 20)))) {
                            //shoot third
                            //if ready to shoot
                            robot.rightFeeder.setPosition(rightFeederUp);
                            robot.leftFeeder.setPosition(leftFeederUp);
                            //robot.deflector.setPosition(1);
                            launchTime = robot.stateTime.seconds();
                            purple2 = true;
                        } else if ((purple1 && !green && !purple2) && ((robot.spinnerLeft.getVelocity() >= (velocity - 20)) && (robot.spinnerLeft.getVelocity() <= (velocity + 20)))) {
                            //shoot second
                            //if ready to shoot
                            robot.leftFeeder.setPosition(leftFeederUp);
                            //robot.deflector.setPosition(deflectorMiddle);
                            launchTime = robot.stateTime.seconds();
                            green = true;
                        } else if ((!purple1 && !green && !purple2) && ((robot.spinnerRight.getVelocity() >= (velocity - 20)) && (robot.spinnerRight.getVelocity() <= (velocity + 20)))) {
                            //shoot first
                            //if ready to shoot
                            robot.rightFeeder.setPosition(rightFeederUp);
                            //robot.deflector.setPosition(deflectorMiddle);
                            launchTime = robot.stateTime.seconds();
                            purple1 = true;
                        }
                    }


                if(((robot.leftFeeder.getPosition() > leftFeederUp-.05) || ((robot.rightFeeder.getPosition() > rightFeederUp-.05))) && (((robot.stateTime.seconds()-launchTime) >= .25) && ((robot.stateTime.seconds()-launchTime) <= .4))){
                    robot.leftFeeder.setPosition(leftFeederDown);
                    robot.rightFeeder.setPosition(rightFeederDown);
                    //robot.deflector.setPosition(deflectorMiddle);
                    if(purple1){
                        robot.intake.setPower(-.2);
                    } else {
                        robot.intake.setPower(-.4);
                    }
                } else if (((robot.stateTime.seconds()-launchTime) >= .4) && (robot.stateTime.seconds()-launchTime) < 1){
                    if(green && purple1){
                        robot.intake.setPower(1);
                    } else{
                        robot.intake.setPower(0);
                    }
                    //robot.deflector.setPosition(1);
                }else if (((robot.stateTime.seconds()-launchTime) >= 1)){
                    if (gamePad1.Right_Trigger.isDown() || gamePad2.Right_Trigger.isDown()){
                        robot.intake.setPower(intakeFast);
                    } else {
                        robot.intake.setPower(0);
                    }

                    if (gamePad1.Left_Trigger.isDown() || gamePad2.Left_Trigger.isDown()){
                        robot.intake.setPower(-intakeFast);
                    }
                    return (purple1 && purple2 && green);
                }

                break;
            case "GPP":
                    if (robot.stateTime.seconds() - launchTime >= 1) {
                        //if not shooting
                        if ((green && purple1 && !purple2) && ((robot.spinnerRight.getVelocity() >= (velocity - 20)) && (robot.spinnerRight.getVelocity() <= (velocity + 20)))) {
                            //shoot third
                            //if ready to shoot
                            robot.rightFeeder.setPosition(rightFeederUp);
                            robot.leftFeeder.setPosition(leftFeederUp);
                            robot.deflector.setPosition(deflectorMiddle);
                            launchTime = robot.stateTime.seconds();
                            purple2 = true;
                        } else if ((green && !purple1 && !purple2) && ((robot.spinnerRight.getVelocity() >= (velocity - 20)) && (robot.spinnerRight.getVelocity() <= (velocity + 20)))) {
                            //shoot second
                            //if ready to shoot
                            robot.rightFeeder.setPosition(rightFeederUp);
                            //robot.deflector.setPosition(1);
                            launchTime = robot.stateTime.seconds();
                            purple1 = true;
                        } else if ((!green && !purple1 && !purple2) && ((robot.spinnerLeft.getVelocity() >= (velocity - 20)) && (robot.spinnerLeft.getVelocity() <= (velocity + 20)))) {
                            //shoot first
                            //if ready to shoot
                            robot.leftFeeder.setPosition(leftFeederUp);
                            //robot.deflector.setPosition(deflectorMiddle);
                            launchTime = robot.stateTime.seconds();
                            green = true;
                        }
                    }


                if(((robot.leftFeeder.getPosition() > leftFeederUp-.05) || ((robot.rightFeeder.getPosition() > rightFeederUp-.05))) && (((robot.stateTime.seconds()-launchTime) >= .25) && ((robot.stateTime.seconds()-launchTime) <= .4))){
                    robot.leftFeeder.setPosition(leftFeederDown);
                    robot.rightFeeder.setPosition(rightFeederDown);
                    //robot.deflector.setPosition(deflectorMiddle);
                    if(green){
                        robot.intake.setPower(0);
                    } else {
                        robot.intake.setPower(-.4);
                    }
                } else if (((robot.stateTime.seconds()-launchTime) >= .4) && (robot.stateTime.seconds()-launchTime) < 1){
                    if(green && purple1){
                        robot.intake.setPower(1);
                    } else{
                        robot.intake.setPower(0);
                    }
                    //robot.deflector.setPosition(1);
                }else if (((robot.stateTime.seconds()-launchTime) >= 1)){
                    if (gamePad1.Right_Trigger.isDown() || gamePad2.Right_Trigger.isDown()){
                        robot.intake.setPower(intakeFast);
                    } else {
                        robot.intake.setPower(0);
                    }

                    if (gamePad1.Left_Trigger.isDown() || gamePad2.Left_Trigger.isDown()){
                        robot.intake.setPower(-intakeFast);
                    }
                    return (purple1 && purple2 && green);
                }

                break;
        }



        return false;
    }



}

