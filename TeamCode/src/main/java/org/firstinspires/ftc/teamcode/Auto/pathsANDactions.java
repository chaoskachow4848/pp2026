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
    public static PathChain drivetoPPG, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3, drivetoPreload, toIntake, Intake1, ShootPickup1, toPGP, intakePGPGreen, intakePGPLast, shootPGP, intakePGPFirst;

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



        toPGP = robot.drive.pathBuilder()
                .addPath(new BezierLine(new Pose(57.500, 17.500), new Pose(40, 59)))
                .setLinearHeadingInterpolation(Math.toRadians(114), Math.toRadians(180))
                .build();
        intakePGPFirst = robot.drive.pathBuilder()
                .addPath(new BezierLine(new Pose(40, 59), new Pose(38, 59)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        intakePGPGreen = robot.drive.pathBuilder()
                .addPath(new BezierLine(new Pose(38, 59), new Pose(33.300, 59)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        intakePGPLast = robot.drive.pathBuilder()
                .addPath(new BezierLine(new Pose(33.300, 59), new Pose(25.600, 59)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        shootPGP = robot.drive.pathBuilder()
                .addPath(new BezierLine(new Pose(25.600, 59), new Pose(57.500, 17.500)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(114))
                .build();

        drivetoPreload = robot.drive.pathBuilder()
                .addPath(new BezierLine(new Pose(64.000, 8.500), new Pose(57.500, 17.700)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(113))
                .build();


        toIntake = robot.drive.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(57.500, 17.700),
                                new Pose(54.000, 31.200),
                                new Pose(37.500, 33)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(113), Math.toRadians(180))
                .build();

        Intake1 = robot.drive.pathBuilder()
                //.addCallback(new DeflectorLeftInGreen(robot))
                .addPath(new BezierLine(new Pose(37.500, 33), new Pose(24.200, 32)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addCallback(new DeflectorRightInPurple(robot))
                //.addPath(new BezierLine(new Pose(25.7, 35.000), new Pose()))
                //.setTangentHeadingInterpolation()
                .build();

        PathChain Intake1Part2 = robot.drive.pathBuilder()
                .addCallback(new DeflectorRightInPurple(robot))
                .addPath(new BezierLine(new Pose(25.7, 35.000), new Pose(24.200, 35.000)))
                .setTangentHeadingInterpolation()
                .build();

        ShootPickup1 = robot.drive.pathBuilder()
                .addPath(new BezierLine(new Pose(24.200, 33), new Pose(57.500, 17.700)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(113))
                .build();

        drivetoPPG = robot.drive.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(57.500, 17.700),
                                new Pose(54.000, 31.200),
                                new Pose(37.500, 35.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(113), Math.toRadians(180))
                .addCallback(new Intaking(robot))
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */



    }
}
