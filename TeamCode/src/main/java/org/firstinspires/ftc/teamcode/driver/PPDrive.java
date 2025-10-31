
package org.firstinspires.ftc.teamcode.driver;

//import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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
    public static double shooterVelocity;
    public static double intakeVelocity = 1;
    public static double difference = 0;
    public static double P = 300;//300;

    public static double I = 0;
    public static double D = 0;//10;
    public static double F = 10;//20;

    public static double deflectorLeftIn = .3583;
    public static double deflectorRightIn = .6683;
    /*
    public static double deflectorLeftIn = .4572;
    public static double deflectorRightIn = .5672;
     */
    public static double deflectorMiddle = .5;
    public static double aimerClose;
    public static double aimerMid = .6;
    public static double aimerFar;
    public static double aimerMin = .25;
    public static double aimerMax = .63;
    public static double shooterMin = 1200;
    public static double shooterMax = 1600;

    public static double shooterFar = shooterMax;
    public static double shooterClose;
    public static double shooterMid = 1500;
    public static double intakeFast = .75;
    public static double intakeSlow = .25;
    public static double leftFeederDown = .2183;
    public static double leftFeederMid = .3183;
    public static double leftFeederUp = .4078;
    public static double rightFeederDown = 0;
    public static double rightFeederMid = .1;
    public static double rightFeederUp = .2;
    public static double aimerPose;
    public static double launchTime;
    public boolean manual = false;
    public boolean powerMode = false;






    button.ButtonReader gamePad1 = new button.ButtonReader();
    button.ButtonReader gamePad2 = new button.ButtonReader();
    /* Declare OpMode members. */
    KachowHardware robot = new KachowHardware();
    state State = state.driving;

    ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() {
        if(kaze.robotPose == null){
            kaze.init(new Pose(72,72,0));//kaze init before robotinit
        }
        robot.init(hardwareMap);
        gamepad2.runLedEffect(robot.redled);
        gamepad1.runLedEffect(robot.blueled);
        telemetry.update();
        robot.pattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
        robot.rightFeeder.setPosition(rightFeederDown);
        robot.leftFeeder.setPosition(leftFeederDown);
        robot.deflector.setPosition(deflectorMiddle);
        waitForStart();
        robot.runtime.reset();
        robot.spinner.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P,I, D, F));
        runtime.reset();
        Vector2d target = new Vector2d(0,144);
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


            robot.kachow.roadRunner.updatePose();
            telemetry.addLine("x: " + robot.kachow.roadRunner.getPose().getX());
            telemetry.addLine("y: " + robot.kachow.roadRunner.getPose().getY());
            telemetry.addLine("aimer: " + aimerPose);
            Vector2d botPoint = new Vector2d(robot.kachow.roadRunner.getPose().getX(), robot.kachow.roadRunner.getPose().getY());
            robot.kachow.bot_to_target = Math.sqrt(Math.abs((botPoint.x-target.x)*(botPoint.x-target.x) + (botPoint.y-target.y)*(botPoint.y-target.y)));
            aimerPose = ((robot.kachow.bot_to_target)/160);
            if (aimerPose>aimerMax){
                aimerPose = aimerMid;
            }else if (aimerPose<aimerMin){
                aimerPose = aimerMin;
            }
            shooterVelocity = 1500*((robot.kachow.bot_to_target)/130);
            if (shooterVelocity>shooterMid){
                shooterVelocity = shooterMid;
            }else if (shooterVelocity<shooterMin){
                shooterVelocity = shooterMin;
            }
            if(botPoint.y <= 72){
                aimerPose = aimerMax;
            }
            if(botPoint.y <= 72){
                shooterVelocity = shooterMax;
            }



            switch (State){
                case driving:
                    robot.spinner.setVelocity(0);
                    robot.intake.setPower(0);

                    robot.kachow.robotCentric(opModeIsActive(), gamepad1, gamepad2, robot);
                    robot.aimer.setPosition(aimerPose);

                    if(gamePad1.Right_Trigger.wasJustPressed()){
                        robot.deflector.setPosition(deflectorRightIn);
                        changeStateTo(state.aimbot);
                        break;
                    }
                    if(gamePad2.Right_Trigger.wasJustPressed()){
                        changeStateTo(state.intaking);
                        break;
                    }
                    break;

                case intaking:
                    if(robot.stateChanged){
                        gamePad2.Right_Bumper.currToggleState = true;
                        gamePad2.Left_Bumper.currToggleState = false;
                        robot.spinner.setVelocity(0);
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

                    if(gamePad2.Right_Bumper.getToggleState()){
                        robot.deflector.setPosition(deflectorRightIn);
                    }else{
                        robot.deflector.setPosition(deflectorLeftIn);
                    }

                    if(gamePad2.Left_Bumper.getToggleState()){
                        robot.spinner.setVelocity(shooterVelocity);
                    } else {
                        robot.spinner.setVelocity(0);
                    }


                    telemetry.addData("Intake: ", robot.intake.getVelocity());
                    telemetry.addData("IntakePower: ", robot.intake.getPower());

                    if(gamePad1.Right_Trigger.wasJustPressed()){
                        robot.intake.setPower(0);
                        //robot.deflector.setPosition(deflectorRightIn);
                        changeStateTo(state.aimbot);
                        break;
                    }
                    break;

                case aimbot:
                    robot.kachow.aimbot(target, gamepad1, gamepad2, robot, .18);
                   /* if (gamePad2.triangle.wasJustPressed()){
                        shooterVelocity = shooterVelocity+100;
                        robot.spinner.setVelocity(shooterVelocity);
                    }
                    if (gamePad2.x.wasJustPressed()){
                        shooterVelocity = shooterVelocity-100;
                        robot.spinner.setVelocity(shooterVelocity);
                    }
                    */

                    if(((robot.spinner.getVelocity() >= (shooterVelocity-60)) || powerMode) && (shooterVelocity > 1400)) {
                        //robot.spinner.setPower(robot.spinner.getPower() + (shooterVelocity-robot.spinner.getVelocity()) * .00001);
                        //powerMode = true;
                        robot.spinner.setVelocity(shooterVelocity);
                    } else if (!powerMode){
                        robot.spinner.setVelocity(shooterVelocity);
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
                    if((robot.spinner.getVelocity() >= (shooterVelocity-20)) && (robot.spinner.getVelocity() <= (shooterVelocity+40))){
                        gamepad2.rumble(50);
                        if (gamePad2.Right_Trigger.wasJustPressed()){
                            robot.rightFeeder.setPosition(rightFeederUp);
                            //robot.deflector.setPosition(deflectorLeftIn);
                            launchTime = robot.stateTime.seconds();
                        }

                        if (gamePad2.Left_Trigger.wasJustPressed()){
                            robot.leftFeeder.setPosition(leftFeederUp);
                            robot.deflector.setPosition(deflectorMiddle);
                            //robot.deflector.setPosition(deflectorRightIn);
                            launchTime = robot.stateTime.seconds();
                        }
                    }

                    if(((robot.leftFeeder.getPosition() > leftFeederUp-.05) || ((robot.rightFeeder.getPosition() > rightFeederUp-.05))) && (((robot.stateTime.seconds()-launchTime) >= .25) && ((robot.stateTime.seconds()-launchTime) <= 1))){
                        robot.leftFeeder.setPosition(leftFeederDown);
                        robot.rightFeeder.setPosition(rightFeederDown);
                        //robot.deflector.setPosition(deflectorMiddle);
                        robot.intake.setPower(-.5);
                    } else if (((robot.stateTime.seconds()-launchTime) >= .4) && (robot.stateTime.seconds()-launchTime) < 1){
                        robot.intake.setPower(1);
                        robot.deflector.setPosition(deflectorRightIn);
                    }else if (((robot.stateTime.seconds()-launchTime) >= 1)){
                        robot.intake.setPower(0);
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



                    robot.spinner.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P,I, D, F));


                    telemetry.addData("spinner: ", robot.spinner.getVelocity());
                    telemetry.addData("spinnerPower: ", robot.spinner.getPower());
                    telemetry.addData("TargetVelocity: ", shooterVelocity);
                    telemetry.addData("P: ", P);
                    telemetry.addData("I: ", I);
                    telemetry.addData("D: ", D);
                    telemetry.addData("F: ", F);

                    if(gamePad1.Right_Trigger.wasJustReleased()){
                        changeStateTo(state.driving);
                        manual = false;
                        powerMode = false;
                    }
                    break;

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
        gamePad2.update(gamepad2, robot);
        robot.stateUpdate(State);
        gamePad1.update(gamepad1, robot);
        if(gamepad1.share && gamepad1.options){
            robot.kachow.roadRunner.setPose(new Pose(60, 60, Math.toRadians(robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)+ kaze.headingOffset)));
        }
        kaze.update(robot.kachow.roadRunner);
        telemetry.update();
        kaze.drawCurrentAndHistory(robot.kachow.roadRunner);
    }
    public void changeStateTo(state tostate){
        State = tostate;
        robot.stateUpdate(State);
    }


}

