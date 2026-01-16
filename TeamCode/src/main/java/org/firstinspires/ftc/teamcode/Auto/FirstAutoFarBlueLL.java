package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.Auto.Motifpaths.Intake1;
import static org.firstinspires.ftc.teamcode.Auto.Motifpaths.ShootPickup1;
import static org.firstinspires.ftc.teamcode.Auto.Motifpaths.drivetoPPG;
import static org.firstinspires.ftc.teamcode.Auto.Motifpaths.drivetoPreload;
import static org.firstinspires.ftc.teamcode.Auto.Motifpaths.intakePGPFirst;
import static org.firstinspires.ftc.teamcode.Auto.Motifpaths.intakePGPGreen;
import static org.firstinspires.ftc.teamcode.Auto.Motifpaths.intakePGPLast;
import static org.firstinspires.ftc.teamcode.Auto.Motifpaths.shootPGP;
import static org.firstinspires.ftc.teamcode.Auto.Motifpaths.toPGP;
import static org.firstinspires.ftc.teamcode.Auto.Motifpaths.toPPG;
import static org.firstinspires.ftc.teamcode.driver.PPDrive.deflectorLeftIn;
import static org.firstinspires.ftc.teamcode.driver.PPDrive.deflectorMiddle;
import static org.firstinspires.ftc.teamcode.driver.PPDrive.deflectorRightIn;
import static org.firstinspires.ftc.teamcode.driver.PPDrive.launchTime;
import static org.firstinspires.ftc.teamcode.driver.PPDrive.leftFeederDown;
import static org.firstinspires.ftc.teamcode.driver.PPDrive.leftFeederUp;
import static org.firstinspires.ftc.teamcode.driver.PPDrive.rightFeederDown;
import static org.firstinspires.ftc.teamcode.driver.PPDrive.rightFeederUp;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.KachowHardware;
import org.firstinspires.ftc.teamcode.hardware.KachowHardware.state;
import org.firstinspires.ftc.teamcode.hardware.kaze;

@Autonomous(name = "MotifFarBlueLL", group = "4848")
public final class FirstAutoFarBlueLL extends LinearOpMode {

    boolean wasMade = false;
    boolean isFirst = true;
    boolean firstScored = false;
    boolean secondScored = false;
    boolean thirdScored = false;
    boolean purple1 = false;
    boolean purple2 = false;
    boolean green = false;
    String pattern;
    int Atag1;
    int Atag2;
    int Atag3;
    state State = state.idle;

    KachowHardware robot = new KachowHardware();
    double angle = 0;
    LLResult result = null;
    private Limelight3A limelight;
    boolean fast = false;


    @Override
    public void runOpMode() throws InterruptedException {
//limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(3);
        double x;
        double y;
        final Pose startPose = new Pose(64.000, 8.500, Math.toRadians(90)); // Start Pose of our robot.
        //launchTime = 0;
        kaze.init(startPose, true);
        Motifpaths actions = new Motifpaths(robot);
        robot.init(hardwareMap);

        robot.imu.resetYaw();
        //actions.down().run(telemetryPacket);
        limelight.start();


        while (!isStarted() && !isStopRequested()) {
            update();
            kaze.drawCurrentAndHistory(robot.kachow.drive);
            actions.buildPaths();
            gamepad2.runLedEffect(robot.redled);
            gamepad1.runLedEffect(robot.blueled);
            telemetry.update();
            robot.rightFeeder.setPosition(rightFeederDown);
            robot.leftFeeder.setPosition(leftFeederDown);
            robot.deflector.setPosition(deflectorRightIn);
            //robot.drive.setStartingPose(startPose);
            robot.drive.setPose(startPose);
            launchTime = 0;



            result = limelight.getLatestResult();
            if(result.getFiducialResults().toArray().length == 3){
                Atag1 = result.getFiducialResults().get(0).getFiducialId();
                Atag2 = result.getFiducialResults().get(1).getFiducialId();
                Atag3 = result.getFiducialResults().get(2).getFiducialId();
                telemetry.addLine("Detected 3");
            } else if (result.getFiducialResults().toArray().length == 2) {
                Atag1 = result.getFiducialResults().get(0).getFiducialId();
                Atag2 = result.getFiducialResults().get(1).getFiducialId();
                Atag3 = result.getFiducialResults().get(1).getFiducialId();
                telemetry.addLine("Detected 2");
            } else if (result.getFiducialResults().toArray().length == 1) {
                Atag1 = result.getFiducialResults().get(0).getFiducialId();
                Atag2 = result.getFiducialResults().get(0).getFiducialId();
                Atag3 = result.getFiducialResults().get(0).getFiducialId();
                telemetry.addLine("Detected 1");
            } else {
                telemetry.addLine("Not Detected");
                Atag1 = 0;
                Atag2 = 0;
                Atag3 = 0;
            }

            if ((Atag1 == 21) || (Atag2 == 21) || (Atag3 == 21)){
                pattern = "GPP";
            } else if ((Atag1 == 22) || (Atag2 == 22) || (Atag3 == 22)){
                pattern = "PGP";
            } else if ((Atag1 == 23) || (Atag2 == 23) || (Atag3 == 23)){
                pattern = "PPG";
            }
            if (pattern == null){
                telemetry.addLine("NO PATTERN DETECTED!!!");
                telemetry.update();
            } else {
                telemetry.addLine(pattern);
            }
        }
        //update();
        //robot.spinner.setVelocity(shooterFar);
        limelight.stop();
        if(pattern == null){
            pattern = "PPG";
        }
        kaze.update(robot.kachow.drive, pattern);
        while (opModeIsActive()) {
            if(fast){
                robot.drive.constants.drivePIDFCoefficients(new FilteredPIDFCoefficients(0.015, 0, 0.005, 0.01, .06));
            } else{
                robot.drive.constants.drivePIDFCoefficients(new FilteredPIDFCoefficients(0.006, 0, 0.001, 0.01, .06));
            }


            update();
            switch (State){
                case idle:
                    if(robot.stateChanged){
                        //bucketScore = actions.scoreSampleBlue(drive, robot.drive.pose);
                    }
                    robot.spinnerLeft.setVelocity(1640);//1640
                    robot.spinnerRight.setVelocity(1640);//1640
                    robot.aimer.setPosition(.55);//.64

                    if(!robot.drive.isBusy()) {
                        robot.drive.followPath(drivetoPreload, true);
                        changeStateTo(state.launchPreload);
                        fast = true;
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
                        if(launch(pattern, 1640)){
                            robot.deflector.setPosition(deflectorMiddle);
                            robot.drive.followPath(drivetoPPG);
                            changeStateTo(state.drivetoPPG);
                            robot.spinnerLeft.setVelocity(1640);
                            robot.spinnerRight.setVelocity(1640);
                            firstScored = true;
                            purple1 = false;
                            purple2 = false;
                            green = false;
                            fast = false;
                            //robot.leftFeeder.setPosition(leftFeederMid);
                            //robot.rightFeeder.setPosition(rightFeederMid/2);
                        }
                    }
                    break;

                case drivetoPPG:
                    if(robot.stateChanged){
                        //firstScore = actions.scoreSampleBlueIntake(drive, robot.drive.pose);
                        robot.deflector.setPosition(deflectorLeftIn);
                        robot.spinnerLeft.setVelocity(1640);
                        robot.spinnerRight.setVelocity(1640);
                    }
                    robot.intake.setPower(.5);

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
                                robot.deflector.setPosition(deflectorRightIn);
                                changeStateTo(state.firstIntakeGreen);
                            }} else {
                            changeStateTo(state.launchPreload);
                        }
                    }
                    break;

                case firstIntakeGreen:
                    robot.intake.setPower(.3);

                    if(!robot.drive.isBusy()) {
                        robot.rightFeeder.setPosition(rightFeederDown);
                        robot.drive.followPath(ShootPickup1, true);
                        changeStateTo(state.launch1);
                        robot.spinnerLeft.setVelocity(1640);
                        robot.spinnerRight.setVelocity(1640);
                        robot.intake.setPower(0);
                        fast = true;
                    }
                    break;

                case launch1:
                    if(robot.stateChanged){
                        //secondSample = actions.intakeFirst(drive, robot.drive.pose, 58.5, 54, -90);
                        telemetry.addLine("in here");
                        //robot.deflector.setPosition(deflectorMiddle);
                        launchTime = 0;

                    }
                    //robot.drive.followPath(scorePickup1);
                    //check for sensor intake
                    if((robot.stateTime.seconds() > 1) && (robot.stateTime.seconds() < 1.2)){
                        robot.deflector.setPosition(deflectorRightIn);
                        robot.intake.setPower(-.2);
                    } else if ((robot.stateTime.seconds() > 1.25)){
                        robot.intake.setPower(0);
                    }
                    if(!robot.drive.isBusy()){
                        if(launch(pattern, 1640)){
                            //robot.rightFeeder.setPosition(rightFeederMid/2);
                            secondScored = true;
                            changeStateTo(state.drivetoPGP);
                            robot.drive.followPath(toPGP);
                            purple1 = false;
                            purple2 = false;
                            green = false;
                            fast = false;
                            //robot.spinner.setPower(0);

                        }

                    }
                    break;

                case drivetoPGP:
                    if(robot.stateChanged){
                        //robot.spinner.setPower(0);
                        robot.deflector.setPosition(deflectorRightIn);

                    }
                    robot.intake.setPower(.4);
                    telemetry.addData("percentage: ", robot.drive.getPathCompletion());
                    if(!robot.drive.isBusy()) {
                        robot.leftFeeder.setPosition(leftFeederDown);
                        robot.rightFeeder.setPosition(rightFeederDown);

                        //robot.rightFeeder.setPosition(rightFeederMid);
                            if (secondScored) {
                                robot.drive.followPath(intakePGPFirst);
                                changeStateTo(state.secondIntakePurple1);
                            }
                    }
                    break;
                case secondIntakePurple1:
                    if(robot.stateChanged){
                        //robot.spinner.setPower(0);
                        robot.deflector.setPosition(deflectorRightIn);
                    }
                    robot.intake.setPower(.4);
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
                    robot.intake.setPower(.4);
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
                        robot.spinnerLeft.setVelocity(1640);
                        robot.spinnerRight.setVelocity(1640);
                        robot.deflector.setPosition(deflectorRightIn);
                    }
                    robot.intake.setPower(.4);
                    telemetry.addData("percentage: ", robot.drive.getPathCompletion());
                    if(!robot.drive.isBusy()) {
                        robot.leftFeeder.setPosition(leftFeederDown);
                        robot.rightFeeder.setPosition(rightFeederDown);

                        //robot.rightFeeder.setPosition(rightFeederMid);
                        if (secondScored) {
                            robot.drive.followPath(shootPGP);
                            robot.deflector.setPosition(deflectorLeftIn);
                            changeStateTo(state.launch2);
                            fast = true;
                        }
                    }
                    break;

                case launch2:
                    if(robot.stateChanged){
                        telemetry.addLine("in here");
                        robot.deflector.setPosition(deflectorRightIn);
                        robot.spinnerLeft.setVelocity(1640);
                        robot.spinnerRight.setVelocity(1640);
                        launchTime = 0;
                    }

                    if(!robot.drive.isBusy()){
                        if(launch(pattern, 1640)){
                            //robot.rightFeeder.setPosition(rightFeederMid/2);
                            //robot.leftFeeder.setPosition(leftFeederMid);
                            thirdScored = true;
                            changeStateTo(state.driving);
                            robot.drive.followPath(toPPG);
                            purple1 = false;
                            purple2 = false;
                            green = false;
                            robot.spinnerLeft.setVelocity(0);
                            robot.spinnerRight.setVelocity(0);
                            robot.intake.setPower(1);
                            fast = false;

                        }

                    } else{
                        if((robot.stateTime.seconds() > 1) && (robot.stateTime.seconds() < 1.2)){
                            robot.deflector.setPosition(deflectorMiddle);
                            robot.intake.setPower(-.3);
                        } else if ((robot.stateTime.seconds() > 1.25)){
                            robot.intake.setPower(0);
                            robot.deflector.setPosition(deflectorRightIn);
                        }
                    }
                    break;
                case driving:
                    robot.leftFeeder.setPosition(leftFeederDown);
                    robot.rightFeeder.setPosition(rightFeederDown);
                    if(robot.stateTime.seconds()>=1){
                        robot.deflector.setPosition(1);
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
    telemetry.addData("isDone: ", !robot.drive.isBusy());
    telemetry.addData("statechange: ", robot.stateChanged);
    telemetry.addData("velocity", robot.spinnerLeft.getVelocity());
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
        robot.spinnerLeft.setVelocity(velocity);
        robot.spinnerRight.setVelocity(velocity);
        switch (pattern) {
            case "PPG":
                if (robot.stateTime.seconds() - launchTime >= 1.3) {
                    //if not shooting
                    if (purple1 && purple2 && !green) {
                        if ((robot.spinnerLeft.getVelocity() >= (velocity - 20)) && (robot.spinnerLeft.getVelocity() <= (velocity + 20))) {
                            //shoot third
                            robot.leftFeeder.setPosition(leftFeederUp);
                            robot.deflector.setPosition(deflectorMiddle);
                            launchTime = robot.stateTime.seconds();
                            green = true;
                        }
                    } else if (purple1 && !purple2 && !green) {
                        if ((robot.spinnerRight.getVelocity() >= (velocity - 20)) && (robot.spinnerRight.getVelocity() <= (velocity + 20))) {
                            //shoot second
                            robot.rightFeeder.setPosition(rightFeederUp);
                            robot.deflector.setPosition(deflectorRightIn);
                            launchTime = robot.stateTime.seconds();
                            purple2 = true;
                        }
                    } else if (!purple1 && !purple2 && !green) {
                        if ((robot.spinnerRight.getVelocity() >= (velocity - 20)) && (robot.spinnerRight.getVelocity() <= (velocity + 20))) {
                            //shoot first
                            robot.rightFeeder.setPosition(rightFeederUp);
                            robot.deflector.setPosition(deflectorRightIn);
                            launchTime = robot.stateTime.seconds();
                            purple1 = true;
                        }
                    }
                }


                if(((robot.leftFeeder.getPosition() > leftFeederUp-.05) || ((robot.rightFeeder.getPosition() > rightFeederUp-.05))) && (((robot.stateTime.seconds()-launchTime) >= .25) && ((robot.stateTime.seconds()-launchTime) <= .4))){
                    robot.leftFeeder.setPosition(leftFeederDown);
                    robot.rightFeeder.setPosition(rightFeederDown);
                    //robot.deflector.setPosition(deflectorMiddle);
                    robot.intake.setPower(-.45);
                } else if (((robot.stateTime.seconds()-launchTime) >= .4) && (robot.stateTime.seconds()-launchTime) < 1){
                    robot.intake.setPower(1);
                    robot.deflector.setPosition(deflectorRightIn);
                }else if (((robot.stateTime.seconds()-launchTime) >= 1)){
                    robot.intake.setPower(0);
                    return (purple1 && purple2 && green);
                }

                break;
            case "PGP":
                if (robot.stateTime.seconds() - launchTime >= 1.3) {
                    //if not shooting
                    if (purple1 && green && !purple2) {
                        if ((robot.spinnerRight.getVelocity() >= (velocity - 20)) && (robot.spinnerRight.getVelocity() <= (velocity + 20))) {
                            //shoot third
                            robot.rightFeeder.setPosition(rightFeederUp);
                            robot.deflector.setPosition(deflectorRightIn);
                            launchTime = robot.stateTime.seconds();
                            purple2 = true;
                        }
                    } else if (purple1 && !green && !purple2) {
                        if ((robot.spinnerLeft.getVelocity() >= (velocity - 20)) && (robot.spinnerLeft.getVelocity() <= (velocity + 20))) {
                            //shoot second
                            robot.leftFeeder.setPosition(leftFeederUp);
                            robot.deflector.setPosition(deflectorMiddle);
                            launchTime = robot.stateTime.seconds();
                            green = true;
                        }
                    } else if (!purple1 && !green && !purple2) {
                        if ((robot.spinnerRight.getVelocity() >= (velocity - 20)) && (robot.spinnerRight.getVelocity() <= (velocity + 20))) {
                            //shoot first
                            robot.rightFeeder.setPosition(rightFeederUp);
                            //robot.deflector.setPosition(deflectorMiddle);
                            launchTime = robot.stateTime.seconds();
                            purple1 = true;
                        }
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
                    robot.deflector.setPosition(deflectorRightIn);
                }else if (((robot.stateTime.seconds()-launchTime) >= 1)){
                    robot.intake.setPower(0);
                    return (purple1 && purple2 && green);
                }

                break;
            case "GPP":
                if (robot.stateTime.seconds() - launchTime >= 1.3) {
                    //if not shooting
                    if (green && purple1 && !purple2) {
                        if ((robot.spinnerRight.getVelocity() >= (velocity - 20)) && (robot.spinnerRight.getVelocity() <= (velocity + 20))) {
                            //shoot third
                            robot.rightFeeder.setPosition(rightFeederUp);
                            robot.deflector.setPosition(deflectorMiddle);
                            launchTime = robot.stateTime.seconds();
                            purple2 = true;
                        }
                    } else if (green && !purple1 && !purple2) {
                        if ((robot.spinnerRight.getVelocity() >= (velocity - 20)) && (robot.spinnerRight.getVelocity() <= (velocity + 20))) {
                            //shoot second
                            robot.rightFeeder.setPosition(rightFeederUp);
                            robot.deflector.setPosition(deflectorRightIn);
                            launchTime = robot.stateTime.seconds();
                            purple1 = true;
                        }
                    } else if (!green && !purple1 && !purple2) {
                        if ((robot.spinnerLeft.getVelocity() >= (velocity - 20)) && (robot.spinnerLeft.getVelocity() <= (velocity + 20))) {
                            //shoot first
                            robot.leftFeeder.setPosition(leftFeederUp);
                            robot.deflector.setPosition(deflectorMiddle);
                            launchTime = robot.stateTime.seconds();
                            green = true;
                        }
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
                    robot.deflector.setPosition(deflectorRightIn);
                }else if (((robot.stateTime.seconds()-launchTime) >= 1)){
                    robot.intake.setPower(0);
                    return (purple1 && purple2 && green);
                }

                break;
        }



        return false;
    }

}
