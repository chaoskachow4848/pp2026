package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.Auto.pathsANDactions.drivetoPPG;
import static org.firstinspires.ftc.teamcode.Auto.pathsANDactions.drivetoPreload;
import static org.firstinspires.ftc.teamcode.Auto.pathsANDactions.scorePickup1;
import static org.firstinspires.ftc.teamcode.Auto.pathsANDactions.scorePreload;
import static org.firstinspires.ftc.teamcode.driver.PPDrive.deflectorMiddle;
import static org.firstinspires.ftc.teamcode.driver.PPDrive.leftFeederDown;
import static org.firstinspires.ftc.teamcode.driver.PPDrive.rightFeederDown;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
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
    state State = state.driving;
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
        boolean grabbing = false;
        boolean clawVert = false;
        final Pose startPose = new Pose(64.000, 8.500, Math.toRadians(90)); // Start Pose of our robot.

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
            robot.deflector.setPosition(deflectorMiddle);
            telemetry.update();
            //robot.drive.setStartingPose(startPose);
            robot.drive.setPose(startPose);

            // Don't burn CPU cycles busy-looping in this sample
        }
        //update();
        //robot.spinner.setVelocity(shooterFar);
        while (opModeIsActive()) {
            update();
            switch (State){
                case driving:
                    if(robot.stateChanged){
                        //bucketScore = actions.scoreSampleBlue(drive, robot.drive.pose);
                    }
                    robot.spinner.setVelocity(1600);

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
                        robot.spinner.setVelocity(0);
                    }
                    telemetry.addData("statetime: ", robot.stateTime);

                    //robot.drive.followPath(scorePreload);
                    //check for sensor intake

                    if(!robot.drive.isBusy()){

                        firstScored = true;
                        changeStateTo(state.drivetoPPG);
                        robot.drive.followPath(drivetoPPG);
                    }
                    break;

                case drivetoPPG:
                    if(robot.stateChanged){
                        //firstScore = actions.scoreSampleBlueIntake(drive, robot.drive.pose);
                    }
                    if(!robot.drive.isBusy()) {
                        if (firstScored) {
                            if (secondScored) {
                                if (thirdScored) {
                                    changeStateTo(state.launch3);
                                } else {
                                    changeStateTo(state.launch2);
                                }
                            } else {
                                changeStateTo(state.aimbot);

                            }} else {
                            changeStateTo(state.launchPreload);
                        }
                    }
                    break;

                case launch1:
                    if(robot.stateChanged){
                        //secondSample = actions.intakeFirst(drive, robot.drive.pose, 58.5, 54, -90);
                        telemetry.addLine("in here");
                    }
                    robot.drive.followPath(scorePickup1);
                    //check for sensor intake

                    if(!robot.drive.isBusy()){

                        secondScored = true;
                        changeStateTo(state.drivetoPGP);
                    }
                    break;

            }



        }
    }
public void update(){
    kaze.drawCurrentAndHistory(robot.kachow.drive);
    robot.drive.update();
    robot.stateUpdate(State);

    //telemetryAprilTag(robot.aprilTag);
    telemetry.addData("x", robot.drive.getPose().getX());
    telemetry.addData("y", robot.drive.getPose().getY());
    telemetry.addData("heading RR", Math.toDegrees(robot.drive.getPose().getHeading()));
    telemetry.addData("State: ", State);
    telemetry.addData("Kaze: ", !robot.drive.isBusy());
    telemetry.addData("statechange: ", robot.stateChanged);
    telemetry.update();

}
    public void changeStateTo(state tostate){
        State = tostate;
        //robot.stateUpdate(State);
    }

}
