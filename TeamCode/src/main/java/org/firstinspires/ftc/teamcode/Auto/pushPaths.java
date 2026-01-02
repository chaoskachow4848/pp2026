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

    public static PathChain pushBot;
    public static PathChain shootPreload;
    public static PathChain intakeFirst;
    public static PathChain openGate;
    public static PathChain shootFirst;
    public static PathChain intakeSecond;
    public static PathChain shootSecond;
    public static PathChain intakeThird;
    public static PathChain shootThird;
    public static PathChain leave;

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
                        new BezierLine(new Pose(80.000, 8.500), new Pose(90, 9.200))
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
                                new Pose(87.925, 91.8),
                                new Pose(127, 83.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(68), Math.toRadians(0))
                .build();
        REDopenGate = robot.drive.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(130.000, 83.000),
                                new Pose(112, 76),
                                new Pose(128, 74)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        REDshootFirst = robot.drive.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(129.196, 74), new Pose(86.000, 80.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(50))
                .build();
        REDintakeSecond = robot.drive.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(86.000, 80.000),
                                new Pose(83.664, 56),
                                new Pose(132.000, 59)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(50), Math.toRadians(0))
                .build();

        REDshootSecond = robot.drive.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(132.000, 59), new Pose(86.000, 80.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(50))
                .build();


        REDintakeThird = robot.drive.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(86.000, 80.000),
                                new Pose(72.000, 32),
                                new Pose(132.000, 35.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(50), Math.toRadians(0))
                .build();

        REDshootThird = robot.drive.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(132, 35.000),
                                new Pose(85, 36.112),
                                new Pose(86.5, 17.500)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(68))
                .build();
                /*.addPath(
                        new BezierLine(new Pose(130.000, 35.000), new Pose(86.000, 80.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(50))
                .build();

                 */

        REDleave = robot.drive.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(86.5, 17.5), new Pose(132.000, 35.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(68), Math.toRadians(0))
                .build();








        /// blue
        pushBot = robot.drive.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(64, 8.5), new Pose(54, 9.2))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                .build();
        shootPreload = robot.drive.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(54, 9.2), new Pose(59.5, 16))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(113))
                .build();
        intakeFirst = robot.drive.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(59.5, 16),
                                new Pose(53, 89.5),
                                new Pose(19, 81.5)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(113), Math.toRadians(180))
                .build();
        openGate = robot.drive.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(19, 81.5),
                                new Pose(34, 74.5),
                                new Pose(18.25, 72.5)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        shootFirst = robot.drive.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(17, 72.5), new Pose(60, 78.5))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(133))
                .build();
        intakeSecond = robot.drive.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(60, 78.5),
                                new Pose(65, 50), //83//52
                                new Pose(14, 57.5)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(133), Math.toRadians(180))
                .build();

        shootSecond = robot.drive.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(14, 57.5), new Pose(60, 78.5))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(133))
                .build();

        /*
                intakeSecond = robot.drive.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(60, 78.5),
                                new Pose(41, 39), //83//52
                                new Pose(17.5, 60)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(133), Math.toRadians(161))
                .build();

        shootSecond = robot.drive.pathBuilder()
                /*.addPath(
                        new BezierLine(new Pose(17.5, 61), new Pose(60, 78.5))
                )
                .setLinearHeadingInterpolation(Math.toRadians(152), Math.toRadians(133))
                .build();
                 //
                .addPath(
                new BezierCurve(
                        new Pose(17.5, 60),
                        new Pose(24, 53), //83//52
                        new Pose(60, 78.5)
                )
        )
                .setLinearHeadingInterpolation(Math.toRadians(161), Math.toRadians(133))
                .build();
                 */


        intakeThird = robot.drive.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(60, 78.5),
                                new Pose(74, 29.5),
                                new Pose(14, 32.5)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(133), Math.toRadians(180))
                .build();

        shootThird = robot.drive.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(14, 32.5),
                                new Pose(61, 34.5),
                                new Pose(59.5, 16)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(113))
                .build();
                /*.addPath(
                        new BezierLine(new Pose(130.000, 35.000), new Pose(86.000, 80.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(133))
                .build();

                 */

        leave = robot.drive.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(59.5, 16), new Pose(14, 33.5))
                )
                .setLinearHeadingInterpolation(Math.toRadians(113), Math.toRadians(180))
                .build();


    }
}
