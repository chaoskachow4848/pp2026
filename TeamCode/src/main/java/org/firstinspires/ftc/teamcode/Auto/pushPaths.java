package org.firstinspires.ftc.teamcode.Auto;


import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.hardware.KachowHardware;

public class pushPaths {

    KachowHardware robot;
    public static final Pose startPoseBlue = new Pose(64, 8.5);
    public static final Pose startPoseRed = new Pose(144-64, 8.5);
   public static final Pose launchMidRangeRed = new Pose(85.8, 74.7, Math.toRadians(49.2));
    public static final Pose launchMidRangeBlue = new Pose(58.2, 74.7, Math.toRadians(130));
    public static PathChain REDpushBot;
    public static PathChain REDshootPreload;
    public static PathChain REDintakeFirst;
    public static PathChain REDopenGate;
    public static PathChain REDshootFirst;
    public static PathChain REDintakeSecond;
    public static PathChain REDshootSecond;
    public static PathChain REDintakeThird;
    public static PathChain REDshootThird;
    public static PathChain REDleave;

    public pushPaths(KachowHardware robot){
        this.robot = robot;
    }
    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */


        /// RED
        REDpushBot = robot.drive.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(80.000, 8.500), new Pose(94.200, 9.200))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                .build();
        REDshootPreload = robot.drive.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(94.200, 9.200), new Pose(86.500, 17.500))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(68))
                .build();
        REDintakeFirst = robot.drive.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(86.500, 17.500),
                                new Pose(87.925, 93.533),
                                new Pose(130.000, 83.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(68), Math.toRadians(0))
                .build();
        REDopenGate = robot.drive.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(130.000, 83.000),
                                new Pose(112.150, 76.037),
                                new Pose(129.196, 72)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        REDshootFirst = robot.drive.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(129.196, 69.757), new Pose(86.000, 80.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(53))
                .build();
        REDintakeSecond = robot.drive.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(86.000, 80.000),
                                new Pose(83.664, 53.607),
                                new Pose(130.000, 60.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(53), Math.toRadians(0))
                .build();

        REDshootSecond = robot.drive.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(130.000, 60.000), new Pose(86.000, 80.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(53))
                .build();


        REDintakeThird = robot.drive.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(86.000, 80.000),
                                new Pose(72.000, 26.243),
                                new Pose(130.000, 35.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(53), Math.toRadians(0))
                .build();

        REDshootThird = robot.drive.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(130.000, 35.000), new Pose(86.000, 80.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(53))
                .build();

        REDleave = robot.drive.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(86.000, 80.000), new Pose(130.000, 35.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(53), Math.toRadians(0))
                .build();


    }
}
