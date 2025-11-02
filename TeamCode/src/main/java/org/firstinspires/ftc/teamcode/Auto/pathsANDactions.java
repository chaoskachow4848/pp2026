package org.firstinspires.ftc.teamcode.Auto;


import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.hardware.KachowHardware;

public class pathsANDactions {

    KachowHardware robot;
    public static final Pose startPose = new Pose(28.5, 128, Math.toRadians(180)); // Start Pose of our robot.
    public static final Pose scorePose = new Pose(60, 85, Math.toRadians(135)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    public static final Pose pickup1Pose = new Pose(37, 121, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    public static final Pose pickup2Pose = new Pose(43, 130, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    public static final Pose pickup3Pose = new Pose(49, 135, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    public static Path scorePreload;
    public static PathChain drivetoPPG, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3, drivetoPreload, toIntake;

    public pathsANDactions (KachowHardware robot){
        this.robot = robot;
    }
    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */



        drivetoPreload = robot.drive.pathBuilder()
                .addPath(new BezierLine(new Pose(64.000, 8.500), new Pose(57.500, 17.700)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(114))
                .build();


        toIntake = robot.drive.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(57.500, 17.700),
                                new Pose(54.000, 31.200),
                                new Pose(37.500, 35.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(114), Math.toRadians(180))
                .build();

        PathChain Intake1 = robot.drive.pathBuilder()
                .addPath(new BezierLine(new Pose(37.500, 35.000), new Pose(24.200, 35.200)))
                .setTangentHeadingInterpolation()
                .build();

        PathChain ShootPickup1 = robot.drive.pathBuilder()
                .addPath(new BezierLine(new Pose(24.200, 35.200), new Pose(57.500, 17.700)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(114))
                .build();

        drivetoPPG = robot.drive.pathBuilder()
                .addPath(new BezierLine(new Pose(57.500, 17.700), new Pose(64.000, 8.500)))
                .setLinearHeadingInterpolation(Math.toRadians(114), Math.toRadians(90))
                .addCallback(new Intaking(robot))
                .addPath(
                        new BezierCurve(
                                new Pose(57.500, 17.700),
                                new Pose(54.000, 31.200),
                                new Pose(37.500, 35.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(114), Math.toRadians(180))
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = robot.drive.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = robot.drive.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = robot.drive.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = robot.drive.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = robot.drive.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();


    }
}
