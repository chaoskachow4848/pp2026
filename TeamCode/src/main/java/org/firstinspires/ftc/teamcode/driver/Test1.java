
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
@TeleOp(name="Test", group="4848")
//@Disabled
public class Test1 extends LinearOpMode {

    public static double distance;
    public static double shooterVelocity = 0;
    public static double intakeVelocity = 0;
    public static double difference = 0;
    public static double P = 300;//300;

    public static double I = 0;
    public static double D = 10;//10;
    public static double F = 20;//20;

    public static double deflectorLeftIn;
    public static double deflectorRightIn;
    public static double deflectorMiddle;
    public static double aimerClose;
    public static double aimerMid;
    public static double aimerFar;
    public static double aimerMin = .25;
    public static double aimerMax = .75;
    public static double shooterMin = 1350;
    public static double shooterMax = 2000;

    public static double shooterFar;
    public static double shooterClose;
    public static double shooterMid;
    public static double intakeFast;
    public static double intakeSlow;
    public static double leftFeederDown;
    public static double leftFeederMid;
    public static double leftFeederUp;
    public static double rightFeederDown;
    public static double rightFeederMid;
    public static double rightFeederUp;
    public static double aimerPose;
    public static double launchTime;
    public boolean manual = false;

    public int testNumber = 1;





    button.ButtonReader gamePad1 = new button.ButtonReader();
    button.ButtonReader gamePad2 = new button.ButtonReader();
    /* Declare OpMode members. */
    KachowHardware robot = new KachowHardware();
    state State = state.aimbot;

    ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() {
        kaze.init(new Pose(72,72,0));//kaze init before robotinit
        robot.init(hardwareMap);
        gamepad2.runLedEffect(robot.redled);
        gamepad1.runLedEffect(robot.blueled);
        telemetry.update();
        robot.pattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
        waitForStart();
        robot.spinner.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P,I, D, F));
        runtime.reset();
        Vector2d target = new Vector2d(0,144);
        robot.aimer.setPosition(.75);






        while (opModeIsActive()) {
            if (gamePad2.Right_Bumper.wasJustPressed()){
                testNumber = testNumber+1;
            }
            if (gamePad2.Left_Bumper.wasJustPressed()){
                testNumber = testNumber-1;
            }
            if (testNumber < 1){
                testNumber = 1;
            }
            if (testNumber > 6){
                testNumber = 6;
            }
            update();
            switch (testNumber){
                case 1:
                    if (gamePad2.Right_Trigger.wasJustPressed()){
                        robot.intake.setPower(robot.intake.getPower()+.01);
                    }
                    if (gamePad2.Left_Trigger.wasJustPressed()){
                        robot.intake.setPower(robot.intake.getPower()-.01);
                    }
                    telemetry.addData("Intake: ", robot.intake.getPower());
                    telemetry.update();
                    break;

                case 2:

                    if (gamePad2.triangle.wasJustPressed()){
                        robot.spinner.setVelocity(1900);
                    }
                    if (gamePad2.Right_Trigger.wasJustPressed()){
                        robot.spinner.setVelocity(robot.spinner.getVelocity()+20);
                    }
                    if (gamePad2.Left_Trigger.wasJustPressed()){
                        robot.spinner.setVelocity(robot.spinner.getVelocity()-20);
                    }
                    telemetry.addData("Spinner: ", robot.spinner.getVelocity());
                    telemetry.addLine("shooter Velocity: " + shooterVelocity);
                    telemetry.update();
                    break;

                case 3:

                    if (gamePad2.Dpad_Up.wasJustPressed()){
                        robot.aimer.setPosition(robot.aimer.getPosition()+.01);
                    }
                    if (gamePad2.Dpad_Down.wasJustPressed()){
                        robot.aimer.setPosition(robot.aimer.getPosition()-.01);
                    }
                    telemetry.addData("Aimer: ", robot.aimer.getPosition());
                    telemetry.addLine("aimer pose: " + aimerPose);
                    telemetry.update();
                    break;

                case 4:

                    if (gamePad2.Dpad_Up.wasJustPressed()){
                        robot.deflector.setPosition(robot.deflector.getPosition()+.01);
                    }
                    if (gamePad2.Dpad_Down.wasJustPressed()){
                        robot.deflector.setPosition(robot.deflector.getPosition()-.01);
                    }
                    telemetry.addData("Deflector: ", robot.deflector.getPosition());
                    telemetry.update();
                    break;


                case 5:

                    if (gamePad2.Dpad_Up.wasJustPressed()){
                        robot.leftFeeder.setPosition(robot.leftFeeder.getPosition()+.01);
                    }
                    if (gamePad2.Dpad_Down.wasJustPressed()){
                        robot.leftFeeder.setPosition(robot.leftFeeder.getPosition()-.01);
                    }
                    telemetry.addData("Left feeder: ", robot.leftFeeder.getPosition());
                    telemetry.update();
                    break;


                case 6:

                    if (gamePad2.Dpad_Up.wasJustPressed()){
                        robot.rightFeeder.setPosition(robot.rightFeeder.getPosition()+.01);
                    }
                    if (gamePad2.Dpad_Down.wasJustPressed()){
                        robot.rightFeeder.setPosition(robot.rightFeeder.getPosition()-.01);
                    }
                    telemetry.addData("Right feeder: ", robot.rightFeeder.getPosition());
                    telemetry.update();
                    break;


            }

            robot.kachow.drive.updatePose();
            telemetry.addLine("x: " + robot.kachow.drive.getPose().getX());
            telemetry.addLine("y: " + robot.kachow.drive.getPose().getY());
            Vector2d botPoint = new Vector2d(robot.kachow.drive.getPose().getX(), robot.kachow.drive.getPose().getY());
            robot.kachow.bot_to_target = Math.sqrt(Math.abs((botPoint.x-target.x)*(botPoint.x-target.x) + (botPoint.y-target.y)*(botPoint.y-target.y)));
            aimerPose = ((robot.kachow.bot_to_target)/110);
            if (aimerPose>aimerMax){
                aimerPose = aimerMax;
            }else if (aimerPose<aimerMin){
                aimerPose = aimerMin;
            }
            shooterVelocity = 1900*((robot.kachow.bot_to_target)/110);
            if (shooterVelocity>shooterMax){
                shooterVelocity = shooterMax;
            }else if (shooterVelocity<shooterMin){
                shooterVelocity = shooterMin;
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
    Vector2d target = new Vector2d(0,144);

    public void update(){ //place once on start of loop
        //robot.kachow.aimbot(target, gamepad1, gamepad2, robot, .13);
        gamePad2.update(gamepad2, robot);
        gamePad1.update(gamepad1, robot);
        if(gamepad1.share && gamepad1.options){
            robot.kachow.drive.setPose(new Pose(60, 60, Math.toRadians(robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES))));
        }
//        kaze.update(robot.kachow.roadRunner);
        kaze.drawCurrentAndHistory(robot.kachow.drive);
    }
    public void changeStateTo(state tostate){
        State = tostate;
        robot.stateUpdate(State);
    }


}

