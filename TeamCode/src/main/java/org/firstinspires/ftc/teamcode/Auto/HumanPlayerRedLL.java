package org.firstinspires.ftc.teamcode.Auto;


import static org.firstinspires.ftc.teamcode.Auto.RiskyPaths.REDintakeClosest;
import static org.firstinspires.ftc.teamcode.Auto.RiskyPaths.REDintakeClustered;
import static org.firstinspires.ftc.teamcode.Auto.RiskyPaths.REDintakeHP;
import static org.firstinspires.ftc.teamcode.Auto.RiskyPaths.REDleave;
import static org.firstinspires.ftc.teamcode.Auto.RiskyPaths.REDshootClustered;
import static org.firstinspires.ftc.teamcode.Auto.RiskyPaths.REDshootFirst;
import static org.firstinspires.ftc.teamcode.Auto.RiskyPaths.REDshootPreload;
import static org.firstinspires.ftc.teamcode.Auto.RiskyPaths.REDshootSecond;
import static org.firstinspires.ftc.teamcode.driver.PPDrive.launchTime;
import static org.firstinspires.ftc.teamcode.driver.PPDrive.leftFeederDown;
import static org.firstinspires.ftc.teamcode.driver.PPDrive.leftFeederUp;
import static org.firstinspires.ftc.teamcode.driver.PPDrive.rightFeederDown;
import static org.firstinspires.ftc.teamcode.driver.PPDrive.rightFeederUp;
import static org.firstinspires.ftc.teamcode.hardware.KachowHardware.state.pushBot;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.KachowHardware;
import org.firstinspires.ftc.teamcode.hardware.KachowHardware.state;
import org.firstinspires.ftc.teamcode.hardware.kaze;

@Autonomous(name = "PushAutoRedLL", group = "4848")
public final class HumanPlayerRedLL extends LinearOpMode {

    boolean wasMade = false;
    boolean isFirst = true;
    boolean singleShoot = false;
    boolean doubleShoot = false;
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
        final Pose startPose = new Pose(144-64.000, 8.500, Math.toRadians(90)); // Start Pose of our robot.
        //launchTime = 0;
        kaze.init(startPose, false);

        RiskyPaths actions = new RiskyPaths(robot);
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

                if (fast) {
                    robot.drive.constants.drivePIDFCoefficients(new FilteredPIDFCoefficients(0.006, 0, 0.001, 0.01, .06));
                } else {
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
                    robot.aimer.setPosition(.58);//.64

                    if(!robot.drive.isBusy()) {
                        robot.drive.followPath(REDshootPreload, true);
                        changeStateTo(state.shootPreload);
                        fast = true;
                    }
                    break;

                case shootPreload:
                    if(robot.stateChanged){
                        //firstScore = actions.scoreSampleBlueIntake(drive, robot.drive.pose);
                        robot.spinnerLeft.setVelocity(1640);
                        robot.spinnerRight.setVelocity(1640);
                    }

                    telemetry.addData("percentage: ", robot.drive.getPathCompletion());
                        if(!robot.drive.isBusy()){
                            if(doubleLaunch(pattern, 1640)){
                                robot.deflector.setPosition(1);
                                robot.drive.followPath(REDintakeClosest);
                                changeStateTo(state.intakeFirst);
                                robot.spinnerLeft.setVelocity(1380);
                                robot.spinnerRight.setVelocity(1380);
                                robot.aimer.setPosition(.52);
                                singleShoot = false;
                                doubleShoot = false;
                                fast = false;
                                //robot.leftFeeder.setPosition(leftFeederMid);
                                //robot.rightFeeder.setPosition(rightFeederMid/2);
                            }
                        }
                    break;

                case intakeFirst:
                    robot.intake.setPower(.5);

                    if(!robot.drive.isBusy()) {
                        robot.rightFeeder.setPosition(rightFeederDown);
                        robot.drive.followPath(REDshootFirst, true);
                        changeStateTo(state.openGate);
                        robot.spinnerLeft.setVelocity(1380);
                        robot.spinnerRight.setVelocity(1380);
                        robot.intake.setPower(.2);
                        fast = true;
                    }
                    break;

                case shootFirst:
                    if(robot.stateChanged){
                        robot.intake.setPower(0);
                        robot.deflector.setPosition(1);

                    }
                    telemetry.addData("percentage: ", robot.drive.getPathCompletion());
                    if(!robot.drive.isBusy()) {
                        if(doubleLaunch(pattern, 1380)){
                            //robot.rightFeeder.setPosition(rightFeederMid/2);
                            changeStateTo(state.intakeSecond);
                            robot.drive.followPath(REDintakeHP);
                            singleShoot = false;
                            doubleShoot = false;
                            fast = false;
                            //robot.spinner.setPower(0);
                            robot.leftFeeder.setPosition(leftFeederDown);
                            robot.rightFeeder.setPosition(rightFeederDown);

                        }
                    }
                    break;
                case intakeSecond:
                    if(robot.stateChanged){
                        //robot.spinner.setPower(0);
                        robot.deflector.setPosition(1);
                    }
                    robot.intake.setPower(.5);
                    telemetry.addData("percentage: ", robot.drive.getPathCompletion());
                    if(!robot.drive.isBusy()) {
                        robot.leftFeeder.setPosition(leftFeederDown);
                        robot.rightFeeder.setPosition(rightFeederDown);
                        //robot.rightFeeder.setPosition(rightFeederMid);
                        robot.drive.followPath(REDshootSecond);
                        robot.deflector.setPosition(1);
                        changeStateTo(state.shootSecond);
                        robot.intake.setPower(.2);

                        fast = true;

                    }
                    break;
                case shootSecond:
                    if(robot.stateChanged){
                        //robot.spinner.setPower(0);
                        robot.deflector.setPosition(1);
                        launchTime = 0;
                    }
                    telemetry.addData("percentage: ", robot.drive.getPathCompletion());
                    if(!robot.drive.isBusy()) {

                        if(doubleLaunch(pattern, 1400)){
                            //robot.rightFeeder.setPosition(rightFeederMid/2);
                            changeStateTo(state.intakeThird);
                            robot.drive.followPath(REDintakeClustered);
                            singleShoot = false;
                            doubleShoot = false;
                            fast = false;
                            //robot.spinner.setPower(0);
                            robot.leftFeeder.setPosition(leftFeederDown);
                            robot.rightFeeder.setPosition(rightFeederDown);

                        }


                    }
                    break;
                case intakeThird:
                    if(robot.stateChanged){
                        robot.spinnerLeft.setVelocity(1640);//1640
                        robot.spinnerRight.setVelocity(1640);//1640
                        robot.aimer.setPosition(.58);
                        robot.deflector.setPosition(1);
                    }
                    robot.intake.setPower(.5);
                    telemetry.addData("percentage: ", robot.drive.getPathCompletion());
                    if(!robot.drive.isBusy()) {
                        robot.leftFeeder.setPosition(leftFeederDown);
                        robot.rightFeeder.setPosition(rightFeederDown);

                        //robot.rightFeeder.setPosition(rightFeederMid);
                            robot.drive.followPath(REDshootClustered);
                            robot.deflector.setPosition(1);
                            changeStateTo(state.shootThird);
                            fast = false;
                        robot.intake.setPower(0);

                    }
                    break;

                case shootThird:
                    robot.intake.setPower(-.5);
                    if(robot.stateChanged){
                        telemetry.addLine("in here");
                        robot.deflector.setPosition(1);
                        robot.spinnerLeft.setVelocity(1640);
                        robot.spinnerRight.setVelocity(1640);
                        launchTime = 0;
                    }

                    if(!robot.drive.isBusy()){
                        if(doubleLaunch(pattern, 1640)){
                            robot.deflector.setPosition(1);
                            changeStateTo(state.leave);
                            robot.drive.followPath(REDleave);
                            singleShoot = false;
                            doubleShoot = false;
                            robot.spinnerLeft.setVelocity(0);
                            robot.spinnerRight.setVelocity(0);
                            robot.intake.setPower(1);
                            fast = true;

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
    telemetry.addData("isDone: ", !robot.drive.isBusy());
    telemetry.addData("statechange: ", robot.stateChanged);
    telemetry.addData("velocity", robot.spinnerLeft.getVelocity());
    telemetry.update();

}
    public void changeStateTo(state tostate){
        State = tostate;
        //robot.stateUpdate(State);
    }

    public boolean doubleLaunch(String pattern, int velocity) {
        //left is green
        //right is purples

        telemetry.addData("launchtime", launchTime);
        if (doubleShoot) {
            telemetry.addLine("doubleshoot");
        }
        if (singleShoot) {
            telemetry.addLine("singleShoot");
        }
        robot.spinnerLeft.setVelocity(velocity);
        robot.spinnerRight.setVelocity(velocity);

                if (robot.stateTime.seconds() - launchTime >= 1.3) {
                    //if not shooting
                    if (doubleShoot && !singleShoot) {
                        if (((robot.spinnerRight.getVelocity() >= (velocity - 20)) && (robot.spinnerRight.getVelocity() <= (velocity + 20))) && ((robot.spinnerLeft.getVelocity() >= (velocity - 20)) && (robot.spinnerLeft.getVelocity() <= (velocity + 20)))) {
                            //shoot third
                            robot.rightFeeder.setPosition(rightFeederUp);
                            robot.leftFeeder.setPosition(leftFeederUp);
                            launchTime = robot.stateTime.seconds();
                            singleShoot = true;
                        }
                    } else if (!doubleShoot & !singleShoot) {
                        if (((robot.spinnerRight.getVelocity() >= (velocity - 20)) && (robot.spinnerRight.getVelocity() <= (velocity + 20))) && ((robot.spinnerLeft.getVelocity() >= (velocity - 20)) && (robot.spinnerLeft.getVelocity() <= (velocity + 20)))) {
                            //shoot first
                            robot.rightFeeder.setPosition(rightFeederUp);
                            robot.leftFeeder.setPosition(leftFeederUp);
                            //robot.deflector.setPosition(deflectorMiddle);
                            launchTime = robot.stateTime.seconds();
                            doubleShoot = true;
                        }
                    }
                }

                if (((robot.leftFeeder.getPosition() > leftFeederUp - .05) || ((robot.rightFeeder.getPosition() > rightFeederUp - .05))) && (((robot.stateTime.seconds() - launchTime) >= .25) && ((robot.stateTime.seconds() - launchTime) <= .4))) {
                    robot.leftFeeder.setPosition(leftFeederDown);
                    robot.rightFeeder.setPosition(rightFeederDown);
                    //robot.deflector.setPosition(deflectorMiddle);
                    if (doubleShoot) {
                        robot.intake.setPower(1);
                    } else {
                        robot.intake.setPower(0);
                    }

                    return (doubleShoot && singleShoot);


                } else if (((robot.stateTime.seconds() - launchTime) >= .4) && (robot.stateTime.seconds() - launchTime) < 1.3) {
                    if (doubleShoot) {
                        robot.intake.setPower(1);
                    } else {
                        robot.intake.setPower(0);
                    }
                    //robot.deflector.setPosition(1);
                } else if (((robot.stateTime.seconds() - launchTime) >= 1.29)) {
                    robot.intake.setPower(0);
                    return (doubleShoot && singleShoot);
                }

        return false;

    }
}
