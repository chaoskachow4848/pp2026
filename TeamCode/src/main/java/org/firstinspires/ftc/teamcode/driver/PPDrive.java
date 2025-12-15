
package org.firstinspires.ftc.teamcode.driver;

//import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

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
@TeleOp(name="PPDrive", group="4848")
//@Disabled
public class PPDrive extends LinearOpMode {

    public static double distance;
    public static Vector2d target;
    public static double shooterVelocity;
    public static double intakeVelocity = 1;
    public static double difference = 0;
    public static double P = 300;//300;

    public static double I = 0;
    public static double D = 0;//10;
    public static double F = 15;//20;

    public static double deflectorLeftIn = .3583-.04;
    public static double deflectorRightIn = .6683-.04;
    /*
    public static double deflectorLeftIn = .4572;
    public static double deflectorRightIn = .5672;
     */
    public static double deflectorMiddle = .5-.04;
    public static double aimerClose;
    public static double aimerMid = .55;
    public static double aimerFar = .65;
    public static double aimerMin = .25;
    public static double aimerMax = .55;
    public static double shooterMin = 1200;
    public static double shooterMax = 1660;

    public static double shooterFar = shooterMax;
    public static double shooterClose;
    public static double shooterMid = 1500;
    public static double intakeFast = .5;
    public static double intakeSlow = .25;
    public static double leftFeederDown = .0;
    public static double leftFeederMid = .06;
    public static double leftFeederUp = .12;
    public static double rightFeederDown = 0;
    public static double rightFeederMid = .06;
    public static double rightFeederUp = .12;
    public static double aimerPose;
    public static double launchTime;
    public boolean manual = false;
    public boolean doubleLaunch = false;
    public boolean powerMode = false;
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
        robot.spinnerLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P,I, D, F));
        robot.spinnerRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P,I, D, F));
        runtime.reset();
        if(kaze.target != null){
            target = kaze.target;
        } else {
            target = new Vector2d(0,144);
        }
        robot.aimer.setPosition(.63);







        while (opModeIsActive()) {
            update();


 /*           double error;
            double errorPower;
            double x;
            double y;
            double turn;
            double heading;

            Vector2d botPoint = new Vector2d(kaze.robotPose.getPose().getX(), kaze.robotPose.getPose().getY());
            Vector2d wallReference = new Vector2d(144, botPoint.y);
            bot_to_target = Math.abs(Math.sqrt((botPoint.x-target.x)*(botPoint.x-target.x) + (botPoint.y-target.y)*(botPoint.y-target.y)));
            bot_to_wall = Math.abs(Math.sqrt(pow(botPoint.x-wallReference.x, 2) + pow(botPoint.y-wallReference.y, 2)));
            wall_to_target = Math.abs(Math.sqrt(pow(wallReference.x-target.x, 2) + pow(wallReference.y-target.y, 2)));

            bot_angle = Math.abs(Math.acos((pow(bot_to_wall,2) + pow(bot_to_target,2) - pow(wall_to_target,2))/
                    (2 * bot_to_wall * bot_to_target)));
            target_angle = Math.abs(Math.asin((bot_to_wall * Math.sin(bot_angle))/wall_to_target));
            wall_angle = Math.abs(Math.toRadians(180) - bot_angle - target_angle);

            if(botPoint.y > target.y){
                bot_angle = -bot_angle;
                target_angle = -target_angle;
                wall_angle = -wall_angle;
            }

            if (robot.runtime.seconds() > 84.8 && robot.runtime.seconds() < 85) {
                gamepad1.rumble(5000);
                gamepad2.rumble(5000);
            }
            if (robot.runtime.seconds() > 109 && robot.runtime.seconds() < 110) {
                gamepad1.rumble(10000);
                gamepad2.rumble(10000);
            }
            heading = robot.kachow.roadRunner.getHeading()+Math.toRadians(kaze.headingOffset);

            x = gamepad1.left_stick_x;
            y = -gamepad1.left_stick_y;

            error = angleDifference(heading, bot_angle);

            errorPower = error / 90;
            if(Math.abs(errorPower)>1){
                errorPower = Math.abs(errorPower)/errorPower;
            }
            if(Math.abs(errorPower)<.1){
                errorPower = (Math.abs(errorPower)/errorPower)*.1;
            }
            if (Math.abs(error) < 1) {
                errorPower = 0;
            }
            turn = -errorPower;


            double botHeading = heading;
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);
            double frontLeftPower = (rotY + rotX + turn) / denominator;
            double backLeftPower = (rotY - rotX + turn) / denominator;
            double frontRightPower = (rotY - rotX - turn) / denominator;
            double backRightPower = (rotY + rotX - turn) / denominator;


            robot.frontright.setPower(frontRightPower);
            robot.frontleft.setPower(frontLeftPower);
            robot.backleft.setPower(backLeftPower);
            robot.backright.setPower(backRightPower);
            //roadRunner.setPose(new Pose(roadRunner.getPose().getX(), roadRunner.getPose().getY(), Math.toRadians(robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)+kaze.headingOffset)));
            kaze.update(robot.kachow.roadRunner);
*/


            robot.kachow.drive.updatePose();
            telemetry.addLine("x: " + robot.kachow.drive.getPose().getX());
            telemetry.addLine("y: " + robot.kachow.drive.getPose().getY());
            telemetry.addLine("heading: " + Math.toDegrees(robot.kachow.drive.getPose().getHeading()));
            telemetry.addLine("aimer: " + aimerPose);
            Vector2d botPoint = new Vector2d(robot.kachow.drive.getPose().getX(), robot.kachow.drive.getPose().getY());
            robot.kachow.bot_to_target = Math.sqrt(Math.abs((botPoint.x-target.x)*(botPoint.x-target.x) + (botPoint.y-target.y)*(botPoint.y-target.y)));
            //aimerPose = .5;//((robot.kachow.bot_to_target)/210);
            if (aimerPose>aimerMax){
                aimerPose = aimerMid;
            }else if (aimerPose<aimerMin){
                aimerPose = aimerMin;
            }
            shooterVelocity = 1380;//1600*((robot.kachow.bot_to_target)/120);//1300
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
                    robot.spinnerLeft.setVelocity(0);
                    robot.spinnerRight.setVelocity(0);
                    robot.intake.setPower(0);

                    robot.kachow.robotCentric(opModeIsActive(), gamepad1, gamepad2, robot);
                    robot.aimer.setPosition(aimerPose);

                    if(gamePad1.Right_Trigger.wasJustPressed()){
                        //robot.deflector.setPosition(deflectorRightIn);
                        changeStateTo(state.aimbot);
                        if(botPoint.y <= 60){
                            doubleLaunch = false;
                        } else {
                            doubleLaunch = true;
                        }
                        break;
                    }
                    if(gamePad2.Right_Trigger.wasJustPressed()){
                        robot.deflector.setPosition(deflectorLeftIn);
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
                    if(robot.stateChanged){
                        /*if(gamePad2.Right_Bumper.getToggleState()){
                            gamePad2.Right_Bumper.currentState = true;
                        }
                        gamePad2.Right_Bumper.currToggleState = false;
                        gamePad2.Left_Bumper.currToggleState = false;

                         */
                        //robot.deflector.setPosition(deflectorRightIn);
                        robot.spinnerLeft.setVelocity(0);
                        robot.spinnerRight.setVelocity(0);
                    }
                    robot.kachow.robotCentric(opModeIsActive(), gamepad1, gamepad2, robot);
                    robot.aimer.setPosition(aimerPose);
                    if (gamePad2.Right_Trigger.isDown()){
                        robot.intake.setPower(intakeFast);
                    } else {
                        robot.intake.setPower(0);
                    }

                    if (gamePad2.Left_Trigger.isDown()){
                        robot.intake.setPower(-intakeFast);
                    }


                    if((robot.deflector.getPosition()<deflectorMiddle) && gamePad2.Right_Bumper.wasJustPressed()){
                        //robot.deflector.setPosition(deflectorRightIn);

                    } else if ((robot.deflector.getPosition()> deflectorMiddle) && gamePad2.Right_Bumper.wasJustPressed()) {
                        robot.deflector.setPosition(deflectorLeftIn);

                    }
                   /* if(gamePad2.Right_Bumper.getToggleState()){
                        robot.deflector.setPosition(deflectorRightIn);
                    }else{
                        robot.deflector.setPosition(deflectorLeftIn);
                    }

                    */

                    if(gamePad2.Left_Bumper.isDown()){
                        robot.spinnerLeft.setVelocity(shooterVelocity);
                        robot.spinnerRight.setVelocity(shooterVelocity);

                    } else {
                        robot.spinnerLeft.setVelocity(0);
                        robot.spinnerRight.setVelocity(0);
                    }


                    telemetry.addData("Intake: ", robot.intake.getVelocity());
                    telemetry.addData("IntakePower: ", robot.intake.getPower());
                    robot.deflector.setPosition(1);
                    if(gamePad1.Right_Trigger.wasJustPressed()){
                        robot.intake.setPower(0);
                        //robot.deflector.setPosition(deflectorRightIn);
                        changeStateTo(state.aimbot);
                        if(botPoint.y <= 60){
                            doubleLaunch = false;
                        } else {
                            doubleLaunch = true;
                        }
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
                        if(botPoint.y <= 60){
                            doubleLaunch = false;
                        } else {
                            doubleLaunch = true;
                        }
                        launchTime = -5;
                    }
                    robot.kachow.aimbot(target, gamepad1, gamepad2, robot, .18);
                    if(gamepad1.right_stick_x != 0){
                        target = new Vector2d(target.x+gamepad1.right_stick_x, target.y);
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

                    if(doubleLaunch) {
                        //robot.spinner.setPower(robot.spinner.getPower() + (shooterVelocity-robot.spinner.getVelocity()) * .00001);
                        //powerMode = true;
                        robot.deflector.setPosition(1);
                        robot.spinnerLeft.setVelocity(shooterVelocity);//+120//+140
                        robot.spinnerRight.setVelocity(shooterVelocity);//+120//+140
                    } else {
                        robot.spinnerLeft.setVelocity(shooterVelocity);
                        robot.spinnerRight.setVelocity(shooterVelocity);
                        //robot.intake.setPower(.5);
                        if (((robot.stateTime.seconds()-launchTime) >= .8) && ((robot.stateTime.seconds()-launchTime) <= 1)){
                            //robot.deflector.setPosition(deflectorMiddle);
                        }
                    }
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
                    /*if(doubleLaunch){
                        if((robot.spinner.getVelocity() >= (shooterVelocity+100)) && (robot.spinner.getVelocity() <= (shooterVelocity+140))){
                            gamepad2.rumble(50);

                            if(robot.stateTime.seconds() - launchTime >= 1){

                                if (gamePad2.Right_Trigger.isDown() || gamePad2.triangle.isDown()){
                                    robot.rightFeeder.setPosition(rightFeederUp);
                                    robot.leftFeeder.setPosition(leftFeederUp);
                                    //robot.deflector.setPosition(deflectorLeftIn);
                                    launchTime = robot.stateTime.seconds();
                                    doubleLaunch = false;
                                }

                                if (gamePad2.Left_Trigger.isDown() || gamePad2.triangle.isDown()){
                                    //robot.deflector.setPosition(deflectorMiddle);
                                    robot.rightFeeder.setPosition(rightFeederUp);
                                    robot.leftFeeder.setPosition(leftFeederUp);
                                    //robot.deflector.setPosition(deflectorRightIn);
                                    launchTime = robot.stateTime.seconds();
                                    doubleLaunch = false;
                                }

                            }
                        }

                    } else {*/


                            if(robot.stateTime.seconds() - launchTime >= 1){
                                if(gamePad2.triangle.isDown()){
                                    doubleLaunch = true;
                                }

                                if((robot.spinnerRight.getVelocity() >= (shooterVelocity-20)) && (robot.spinnerRight.getVelocity() <= (shooterVelocity+20))) {
                                    gamepad2.rumble(50);
                                    if (gamePad2.Right_Trigger.isDown() || gamePad2.triangle.isDown()) {
                                        robot.rightFeeder.setPosition(rightFeederUp);
                                        //robot.deflector.setPosition(deflectorLeftIn);
                                        launchTime = robot.stateTime.seconds();
                                    }
                                }

                                if((robot.spinnerLeft.getVelocity() >= (shooterVelocity-20)) && (robot.spinnerLeft.getVelocity() <= (shooterVelocity+20))) {
                                    gamepad2.rumble(50);
                                    if (gamePad2.Left_Trigger.isDown() || gamePad2.triangle.isDown()) {
                                        //robot.deflector.setPosition(deflectorMiddle);
                                        robot.leftFeeder.setPosition(leftFeederUp);
                                        //robot.deflector.setPosition(deflectorRightIn);
                                        launchTime = robot.stateTime.seconds();
                                    }
                                }

                            }





                    if(!robot.stateChanged){
                        if(((robot.stateTime.seconds()-launchTime) >= .25) && ((robot.stateTime.seconds()-launchTime) <= .5)) {
                        if (robot.deflector.getPosition() < deflectorMiddle+.1) {
                            robot.leftFeeder.setPosition(leftFeederUp);
                        }
                    }

                        if(((robot.leftFeeder.getPosition() > leftFeederUp-.05) || ((robot.rightFeeder.getPosition() > rightFeederUp-.05))) && (((robot.stateTime.seconds()-launchTime) >= .25) && ((robot.stateTime.seconds()-launchTime) <= .4))){
                        robot.rightFeeder.setPosition(rightFeederDown);
                        //robot.deflector.setPosition(deflectorMiddle);
                        robot.leftFeeder.setPosition(leftFeederDown);
                        robot.intake.setPower(-.3);
                    } else if (((robot.stateTime.seconds()-launchTime) > .4) && (robot.stateTime.seconds()-launchTime) < .8){
                            robot.intake.setPower(1);
                            robot.leftFeeder.setPosition(leftFeederDown);
                            robot.rightFeeder.setPosition(rightFeederDown);
                        // robot.deflector.setPosition(deflectorRightIn);
                        }else if (((robot.stateTime.seconds()-launchTime) >= .8) && ((robot.stateTime.seconds()-launchTime) <= 1)){
                        robot.intake.setPower(0);
                    } else {
                            if (gamePad2.Right_Trigger.isDown()) {
                            robot.intake.setPower(intakeFast);
                        } else {
                                robot.intake.setPower(0);
                            }

                            if (gamePad2.Left_Trigger.isDown()) {
                                robot.intake.setPower(-intakeFast);
                            }
                        }
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



                    robot.spinnerLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P,I, D, F));
                    robot.spinnerRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P,I, D, F));

                    telemetry.addData("spinner: ", robot.spinnerLeft.getVelocity());
                    telemetry.addData("spinnerPower: ", robot.spinnerLeft.getPower());
                    telemetry.addData("TargetVelocity: ", shooterVelocity);
                    telemetry.addData("P: ", P);
                    telemetry.addData("I: ", I);
                    telemetry.addData("D: ", D);
                    telemetry.addData("F: ", F);
                    if(gamePad1.Right_Trigger.wasJustReleased()){
                        robot.rightFeeder.setPosition(rightFeederDown);
                        robot.leftFeeder.setPosition(leftFeederDown);
                        changeStateTo(state.driving);
                        manual = false;
                        powerMode = false;
                        doubleLaunch = false;
                        launchTime = 0;
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
            //gamepad2.rumble(5000);
        }
        if(runtime.seconds() > 109 && runtime.seconds() < 110){
            gamepad1.rumble(10000);
            //gamepad2.rumble(10000);
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
    }


}

