package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.Auto.pathsANDactions.Intake1;
import static org.firstinspires.ftc.teamcode.Auto.pathsANDactions.ShootPickup1;
import static org.firstinspires.ftc.teamcode.Auto.pathsANDactions.drivetoPPG;
import static org.firstinspires.ftc.teamcode.Auto.pathsANDactions.drivetoPreload;
import static org.firstinspires.ftc.teamcode.Auto.pathsANDactions.intakePGPFirst;
import static org.firstinspires.ftc.teamcode.Auto.pathsANDactions.intakePGPGreen;
import static org.firstinspires.ftc.teamcode.Auto.pathsANDactions.intakePGPLast;
import static org.firstinspires.ftc.teamcode.Auto.pathsANDactions.scorePickup1;
import static org.firstinspires.ftc.teamcode.Auto.pathsANDactions.shootPGP;
import static org.firstinspires.ftc.teamcode.Auto.pathsANDactions.toPGP;
import static org.firstinspires.ftc.teamcode.driver.PPDrive.aimerFar;
import static org.firstinspires.ftc.teamcode.driver.PPDrive.deflectorLeftIn;
import static org.firstinspires.ftc.teamcode.driver.PPDrive.deflectorMiddle;
import static org.firstinspires.ftc.teamcode.driver.PPDrive.deflectorRightIn;
import static org.firstinspires.ftc.teamcode.driver.PPDrive.launchTime;
import static org.firstinspires.ftc.teamcode.driver.PPDrive.leftFeederDown;
import static org.firstinspires.ftc.teamcode.driver.PPDrive.leftFeederMid;
import static org.firstinspires.ftc.teamcode.driver.PPDrive.leftFeederUp;
import static org.firstinspires.ftc.teamcode.driver.PPDrive.rightFeederDown;
import static org.firstinspires.ftc.teamcode.driver.PPDrive.rightFeederMid;
import static org.firstinspires.ftc.teamcode.driver.PPDrive.rightFeederUp;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.KachowHardware;
import org.firstinspires.ftc.teamcode.hardware.KachowHardware.state;

import org.firstinspires.ftc.teamcode.hardware.kaze;

@Autonomous(name = "FirstAutoFarBlue", group = "4848")
public final class FirstAutoFarBlue extends LinearOpMode {

    boolean wasMade = false;
    boolean isFirst = true;
    boolean firstScored = false;
    boolean secondScored = false;
    boolean thirdScored = false;
    boolean purple1 = false;
    boolean purple2 = false;
    boolean green = false;
    String pattern = "PPG";
    state State = state.idle;

    KachowHardware robot = new KachowHardware();
    double angle = 0;
    LLResult result = null;
    //private Limelight3A limelight;


    @Override
    public void runOpMode() throws InterruptedException {
//limelight
        //limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        //limelight.pipelineSwitch(0);
        double x;
        double y;
        final Pose startPose = new Pose(64.000, 8.500, Math.toRadians(90)); // Start Pose of our robot.
        //launchTime = 0;
        kaze.init(startPose);
        pathsANDactions actions = new pathsANDactions(robot);
        robot.init(hardwareMap);

        robot.imu.resetYaw();
        //actions.down().run(telemetryPacket);
        //limelight.start();


        while (!isStarted() && !isStopRequested()) {
            update();
            kaze.drawCurrentAndHistory(robot.kachow.drive);
            /*LLStatus status = limelight.getStatus();
            telemetry.addData("Name", "%s",
                    status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(),(int)status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());
            telemetry.addData("isGrabbing: ", grabbing);

            telemetry.addData("null: ", result == null);
            telemetry.addData("null2: ", limelight.getLatestResult() == null);

            result = limelight.getLatestResult();

             */
            actions.buildPaths();
            gamepad2.runLedEffect(robot.redled);
            gamepad1.runLedEffect(robot.blueled);
            telemetry.update();
            robot.pattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
            robot.rightFeeder.setPosition(rightFeederDown);
            robot.leftFeeder.setPosition(leftFeederDown);
            robot.deflector.setPosition(deflectorRightIn);
            telemetry.update();
            //robot.drive.setStartingPose(startPose);
            robot.drive.setPose(startPose);
            launchTime = 0;

            // Don't burn CPU cycles busy-looping in this sample
        }
        //update();
        //robot.spinner.setVelocity(shooterFar);
        while (opModeIsActive()) {
            update();
            switch (State){
                case idle:
                    if(robot.stateChanged){
                        //bucketScore = actions.scoreSampleBlue(drive, robot.drive.pose);
                    }
                    robot.spinner.setVelocity(1580);
                    robot.aimer.setPosition(aimerFar);

                    if(!robot.drive.isBusy()) {
                        robot.drive.followPath(drivetoPreload, true);
                        changeStateTo(state.launchPreload);
                    }
                    break;
                case launchPreload:
                    if(robot.stateChanged){
                        //firstSample = new MecanumDrive.FailoverAction(actions.intakeFirst(drive, robot.drive.pose, 45.5, 54, -90), actions.isDone());
                        telemetry.addLine("in here");
                        telemetry.update();
                    }
                    telemetry.addData("statetime: ", robot.stateTime);



                    if(!robot.drive.isBusy()){
                        if(launch(pattern, 1580)){
                            robot.deflector.setPosition(deflectorMiddle);
                            robot.drive.followPath(drivetoPPG);
                            changeStateTo(state.drivetoPPG);
                            robot.spinner.setVelocity(1580);
                            firstScored = true;
                            purple1 = false;
                            purple2 = false;
                            green = false;
                            robot.leftFeeder.setPosition(leftFeederMid);
                            //robot.rightFeeder.setPosition(rightFeederMid/2);
                        }
                    }
                    break;

                case drivetoPPG:
                    if(robot.stateChanged){
                        //firstScore = actions.scoreSampleBlueIntake(drive, robot.drive.pose);
                        robot.deflector.setPosition(deflectorLeftIn);
                        robot.spinner.setVelocity(1580);
                    }

                    telemetry.addData("percentage: ", robot.drive.getPathCompletion());
                    if(!robot.drive.isBusy()) {
                        robot.leftFeeder.setPosition(leftFeederDown);
                       // robot.rightFeeder.setPosition(rightFeederMid/2);
                        if (firstScored) {
                            if (secondScored) {
                                if (thirdScored) {
                                    changeStateTo(state.launch3);
                                } else {
                                    changeStateTo(state.launch2);
                                }
                            } else {
                                robot.drive.followPath(Intake1);
                                robot.deflector.setPosition(deflectorLeftIn);
                                changeStateTo(state.firstIntakeGreen);
                            }} else {
                            changeStateTo(state.launchPreload);
                        }
                    }
                    break;

                case firstIntakeGreen:
                    robot.intake.setPower(.5);

                    if(!robot.drive.isBusy()) {
                        robot.rightFeeder.setPosition(rightFeederDown);
                        robot.drive.followPath(ShootPickup1, true);
                        changeStateTo(state.launch1);
                        robot.spinner.setVelocity(1580);
                        robot.intake.setPower(0);
                    }
                    break;

                case launch1:
                    if(robot.stateChanged){
                        //secondSample = actions.intakeFirst(drive, robot.drive.pose, 58.5, 54, -90);
                        telemetry.addLine("in here");
                        robot.deflector.setPosition(deflectorMiddle);
                        launchTime = 0;

                    }
                    //robot.drive.followPath(scorePickup1);
                    //check for sensor intake
                    if(robot.stateTime.seconds()>1){
                        robot.deflector.setPosition(deflectorRightIn);
                    }
                    if(!robot.drive.isBusy()){
                        if(launch(pattern, 1580)){
                            //robot.rightFeeder.setPosition(rightFeederMid/2);
                            robot.leftFeeder.setPosition(leftFeederMid);
                            secondScored = true;
                            changeStateTo(state.drivetoPGP);
                            robot.drive.followPath(toPGP);
                            purple1 = false;
                            purple2 = false;
                            green = false;
                            //robot.spinner.setPower(0);

                        }

                    }
                    break;

                case drivetoPGP:
                    if(robot.stateChanged){
                        //robot.spinner.setPower(0);
                        robot.deflector.setPosition(deflectorRightIn);

                    }
                    robot.intake.setPower(.5);
                    telemetry.addData("percentage: ", robot.drive.getPathCompletion());
                    if(!robot.drive.isBusy()) {
                        robot.leftFeeder.setPosition(leftFeederDown);
                        robot.rightFeeder.setPosition(rightFeederDown);

                        //robot.rightFeeder.setPosition(rightFeederMid);
                            if (secondScored) {
                                robot.drive.followPath(intakePGPFirst);
                                robot.deflector.setPosition(deflectorLeftIn);
                                changeStateTo(state.secondIntakePurple1);
                            }
                    }
                    break;
                case secondIntakePurple1:
                    if(robot.stateChanged){
                        //robot.spinner.setPower(0);
                        robot.deflector.setPosition(deflectorRightIn);
                    }
                    robot.intake.setPower(.5);
                    telemetry.addData("percentage: ", robot.drive.getPathCompletion());
                    if(!robot.drive.isBusy()) {
                        robot.leftFeeder.setPosition(leftFeederDown);
                        robot.rightFeeder.setPosition(rightFeederDown);

                        //robot.rightFeeder.setPosition(rightFeederMid);
                        if (secondScored) {
                            robot.drive.followPath(intakePGPGreen);
                            robot.deflector.setPosition(deflectorLeftIn);
                            changeStateTo(state.secondIntakeGreen);
                        }
                    }
                    break;
                case secondIntakeGreen:
                    if(robot.stateChanged){
                        //robot.spinner.setPower(0);
                        robot.deflector.setPosition(deflectorLeftIn);
                    }
                    robot.intake.setPower(.5);
                    telemetry.addData("percentage: ", robot.drive.getPathCompletion());
                    if(!robot.drive.isBusy()) {
                        robot.leftFeeder.setPosition(leftFeederDown);
                        robot.rightFeeder.setPosition(rightFeederDown);

                        //robot.rightFeeder.setPosition(rightFeederMid);
                        if (secondScored) {
                            robot.drive.followPath(intakePGPLast);
                            robot.deflector.setPosition(deflectorRightIn);
                            changeStateTo(state.secondIntakeLastPurple);
                        }
                    }
                    break;
                case secondIntakeLastPurple:
                    if(robot.stateChanged){
                        robot.spinner.setVelocity(1580);
                        robot.deflector.setPosition(deflectorRightIn);
                    }
                    robot.intake.setPower(.5);
                    telemetry.addData("percentage: ", robot.drive.getPathCompletion());
                    if(!robot.drive.isBusy()) {
                        robot.leftFeeder.setPosition(leftFeederDown);
                        robot.rightFeeder.setPosition(rightFeederDown);

                        //robot.rightFeeder.setPosition(rightFeederMid);
                        if (secondScored) {
                            robot.drive.followPath(shootPGP);
                            robot.deflector.setPosition(deflectorLeftIn);
                            changeStateTo(state.launch2);
                        }
                    }
                    break;

                case launch2:
                    if(robot.stateChanged){
                        telemetry.addLine("in here");
                        robot.deflector.setPosition(deflectorMiddle);
                        robot.spinner.setVelocity(1580);
                        launchTime = 0;
                    }
                    //robot.drive.followPath(scorePickup1);
                    //check for sensor intake
                    if(robot.stateTime.seconds()>1){
                        robot.deflector.setPosition(deflectorRightIn);
                    }
                    if(!robot.drive.isBusy()){
                        if(launch(pattern, 1580)){
                            //robot.rightFeeder.setPosition(rightFeederMid/2);
                            robot.leftFeeder.setPosition(leftFeederMid);
                            thirdScored = true;
                            changeStateTo(state.driving);
                            robot.drive.followPath(toPGP);
                            purple1 = false;
                            purple2 = false;
                            green = false;
                            robot.spinner.setPower(0);

                        }

                    }
                    break;




            }



        }
    }
public void update(){
    kaze.drawCurrentAndHistory(robot.kachow.drive);
    robot.drive.update();
    robot.stateUpdate(State);
    kaze.update(robot.kachow.drive);

    //telemetryAprilTag(robot.aprilTag);
    telemetry.addData("x", robot.drive.getPose().getX());
    telemetry.addData("y", robot.drive.getPose().getY());
    telemetry.addData("heading RR", Math.toDegrees(robot.drive.getPose().getHeading()));
    telemetry.addData("State: ", State);
    telemetry.addData("isDon: ", !robot.drive.isBusy());
    telemetry.addData("statechange: ", robot.stateChanged);
    telemetry.addData("velocity", robot.spinner.getVelocity());
    telemetry.update();

}
    public void changeStateTo(state tostate){
        State = tostate;
        //robot.stateUpdate(State);
    }

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
        robot.spinner.setVelocity(velocity);
            switch (pattern) {
                case "PPG":
                    if ((robot.spinner.getVelocity() >= (velocity - 20)) && (robot.spinner.getVelocity() <= (velocity + 20))) {
                        //if ready to shoot
                        if (robot.stateTime.seconds() - launchTime >= 1) {
                            //if not shooting
                            if (purple1 && purple2 && !green) {
                                //shoot third
                                robot.leftFeeder.setPosition(leftFeederUp);
                                robot.deflector.setPosition(deflectorMiddle);
                                launchTime = robot.stateTime.seconds();
                                green = true;
                            } else if (purple1 && !purple2 && !green) {
                                //shoot second
                                robot.rightFeeder.setPosition(rightFeederUp);
                                robot.deflector.setPosition(deflectorRightIn);
                                launchTime = robot.stateTime.seconds();
                                purple2 = true;
                            } else if (!purple1 && !purple2 && !green) {
                                //shoot first
                                robot.rightFeeder.setPosition(rightFeederUp);
                                robot.deflector.setPosition(deflectorRightIn);
                                launchTime = robot.stateTime.seconds();
                                purple1 = true;
                            }
                        }
                    }
                    break;
                case "PGP":
                    if ((robot.spinner.getVelocity() >= (velocity - 20)) && (robot.spinner.getVelocity() <= (velocity + 20))) {
                        //if ready to shoot
                        if (robot.stateTime.seconds() - launchTime >= 1) {
                            //if not shooting
                            if (purple1 && green && !purple2) {
                                //shoot third
                                robot.rightFeeder.setPosition(rightFeederUp);
                                robot.deflector.setPosition(deflectorRightIn);
                                launchTime = robot.stateTime.seconds();
                                purple2 = true;
                            } else if (purple1 && !green && !purple2) {
                                //shoot second
                                robot.leftFeeder.setPosition(leftFeederUp);
                                robot.deflector.setPosition(deflectorMiddle);
                                launchTime = robot.stateTime.seconds();
                                green = true;
                            } else if (!purple1 && !green && !purple2) {
                                //shoot first
                                robot.rightFeeder.setPosition(rightFeederUp);
                                robot.deflector.setPosition(deflectorRightIn);
                                launchTime = robot.stateTime.seconds();
                                purple1 = true;
                            }
                        }
                    }
                    break;
                case "GPP":
                    if ((robot.spinner.getVelocity() >= (velocity - 20)) && (robot.spinner.getVelocity() <= (velocity + 20))) {
                        //if ready to shoot
                        if (robot.stateTime.seconds() - launchTime >= 1) {
                            //if not shooting
                            if (green && purple1 && !purple2) {
                                //shoot third
                                robot.rightFeeder.setPosition(rightFeederUp);
                                robot.deflector.setPosition(deflectorRightIn);
                                launchTime = robot.stateTime.seconds();
                                purple2 = true;
                            } else if (green && !purple1 && !purple2) {
                                //shoot second
                                robot.rightFeeder.setPosition(rightFeederUp);
                                robot.deflector.setPosition(deflectorRightIn);
                                launchTime = robot.stateTime.seconds();
                                purple1 = true;
                            } else if (!green && !purple1 && !purple2) {
                                //shoot first
                                robot.leftFeeder.setPosition(leftFeederUp);
                                robot.deflector.setPosition(deflectorMiddle);
                                launchTime = robot.stateTime.seconds();
                                green = true;
                            }
                        }
                    }
                    break;
            }

            if(((robot.leftFeeder.getPosition() > leftFeederUp-.05) || ((robot.rightFeeder.getPosition() > rightFeederUp-.05))) && (((robot.stateTime.seconds()-launchTime) >= .25) && ((robot.stateTime.seconds()-launchTime) <= 1))){
                robot.leftFeeder.setPosition(leftFeederDown);
                robot.rightFeeder.setPosition(rightFeederDown);
                //robot.deflector.setPosition(deflectorMiddle);
                robot.intake.setPower(-.6);
            } else if (((robot.stateTime.seconds()-launchTime) >= .4) && (robot.stateTime.seconds()-launchTime) < 1){
                robot.intake.setPower(1);
                robot.deflector.setPosition(deflectorRightIn);
            }else if (((robot.stateTime.seconds()-launchTime) >= 1)){
                robot.intake.setPower(0);
                return (purple1 && purple2 && green);
            }

        return false;
    }

}
