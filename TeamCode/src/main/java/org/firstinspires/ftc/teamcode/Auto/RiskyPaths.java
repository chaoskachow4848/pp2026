package org.firstinspires.ftc.teamcode.Auto;


import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.hardware.KachowHardware;

public class RiskyPaths {

    KachowHardware robot;
    public static final Pose startPoseBlue = new Pose(64, 8.5);
    public static final Pose startPoseRed = new Pose(144-64, 8.5);
    public static final Pose launchFarBlue = new Pose(57.500, 16.7);
    public static final Pose launchFarRed = new Pose(144-57.500, 17.500);
    public static PathChain REDshootPreload;
    public static PathChain REDintakeClosest;
    public static PathChain REDshootFirst;
    public static PathChain REDintakeHP;
    public static PathChain REDshootSecond;
    public static PathChain REDintakeClustered;
    public static PathChain REDshootClustered;
    public static PathChain REDleave;

    public static PathChain shootPreload;
    public static PathChain intakeClosest;
    public static PathChain shootFirst;
    public static PathChain intakeHP1;
    public static PathChain intakeHP2;
    public static PathChain intakeHP3;
    public static PathChain intakeHP4;
    public static PathChain intakeHP5;
    public static PathChain shootSecond;
    public static PathChain intakeClustered1;
    public static PathChain intakeClustered2;
    public static PathChain shootClustered;
    public static PathChain leave;

    public RiskyPaths(KachowHardware robot){
        this.robot = robot;
    }
    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */


        /// RED
        REDshootPreload = robot.drive.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(144-64.000, 8.500), new Pose(144-57.500, 16.7))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(68))
                .build();
        REDintakeClosest = robot.drive.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(144-57.500, 16.7),
                                new Pose(144-44.600, 39.300),
                                new Pose(144-14.000, 32.500)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(68), Math.toRadians(0))
                .build();
        REDshootFirst = robot.drive.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(144-14.000, 32.500), new Pose(144-59.500, 16.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(68))
                .build();
        REDintakeHP = robot.drive.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(144-59.500, 16.000), new Pose(144-15, 16.7))
                )
                .setLinearHeadingInterpolation(Math.toRadians(68), Math.toRadians(205))
                .addPath(
                        new BezierLine(new Pose(144-15, 16.7), new Pose(144-15, 12))
                )
                .setConstantHeadingInterpolation(Math.toRadians(205))
                .addPath(
                        new BezierLine(new Pose(144-15, 12), new Pose(144-15, 12))
                )
                .setConstantHeadingInterpolation(Math.toRadians(160))
                .addPath(
                        new BezierLine(new Pose(144-15, 12), new Pose(144-18.000, 12))
                )
                .setLinearHeadingInterpolation(Math.toRadians(160), Math.toRadians(0))
                .addPath(
                        new BezierLine(new Pose(144-18.000, 12), new Pose(144-15, 12))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        REDshootSecond = robot.drive.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(144-15, 12), new Pose(144-59.500, 16.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(68))
                .build();

        REDintakeClustered = robot.drive.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(144-59.500, 16.000), new Pose(144-11.500, 11.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(68), Math.toRadians(150))
                .addPath(
                        new BezierLine(new Pose(144-11.500, 11.000), new Pose(144-11.500, 32.2))
                )
                .setConstantHeadingInterpolation(Math.toRadians(150))
                .build();

        REDshootClustered = robot.drive.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(144-11.500, 32.2), new Pose(144-59.500, 16.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(150), Math.toRadians(68))
                .build();

        REDleave = robot.drive.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(144-59.5, 16), new Pose(144-14, 33.5))
                )
                .setLinearHeadingInterpolation(Math.toRadians(68), Math.toRadians(0))
                .build();








        /// blue7
        shootPreload = robot.drive.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(64.000, 8.500), new Pose(57.500, 16.7))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(113))
                .build();
        intakeClosest = robot.drive.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(57.500, 16.7),
                                new Pose(44.600, 37),
                                new Pose(14.000, 30)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(113), Math.toRadians(180))
                .build();
        shootFirst = robot.drive.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(14.000, 30), new Pose(59.500, 16.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(113))
                .build();
        intakeHP1 = robot.drive.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(59.500, 16.000), new Pose(15, 16.7))
                )
                .setLinearHeadingInterpolation(Math.toRadians(113), Math.toRadians(205))
                .build();

        intakeHP2 = robot.drive.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(15, 16.7), new Pose(15, 12))
                )
                .setLinearHeadingInterpolation(Math.toRadians(205), Math.toRadians(205))
                .build();

        intakeHP3 = robot.drive.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(15, 12), new Pose(15.5, 11.5))
                )
                .setLinearHeadingInterpolation(Math.toRadians(205), Math.toRadians(160))
                .build();

        intakeHP4 = robot.drive.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(15.5, 11.5), new Pose(25, 12))
                )
                .setLinearHeadingInterpolation(Math.toRadians(160), Math.toRadians(180))
                .build();

        intakeHP5 = robot.drive.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(25, 12), new Pose(15, 12))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        shootSecond = robot.drive.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(15, 12), new Pose(59.500, 16.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(113))
                .build();

        intakeClustered1 = robot.drive.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(59.500, 16.000), new Pose(15, 11.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(113), Math.toRadians(140))
                .build();
        intakeClustered2 = robot.drive.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(15, 11.000), new Pose(15, 32.2))
                )
                .setConstantHeadingInterpolation(Math.toRadians(140))
                .build();

        shootClustered = robot.drive.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(11.500, 32.2), new Pose(59.500, 16.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(150), Math.toRadians(113))
                .build();

        leave = robot.drive.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(59.5, 16), new Pose(14, 33.5))
                )
                .setLinearHeadingInterpolation(Math.toRadians(113), Math.toRadians(180))
                .build();


    }
}
