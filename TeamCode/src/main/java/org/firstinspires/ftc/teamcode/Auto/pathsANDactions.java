package org.firstinspires.ftc.teamcode.Auto;


import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.hardware.KachowHardware;

public class pathsANDactions {

    KachowHardware robot;
    public static final Pose startPoseBlue = new Pose(64, 8.5);
    public static final Pose startPoseRed = new Pose(144-64, 8.5);
    public static final Pose launchFarBlue = new Pose(57.500, 17.700);
    public static final Pose launchFarRed = new Pose(144-57.500, 17.500);
    public static final Pose launchMidRangeRed = new Pose(85.8, 74.7, Math.toRadians(49.2));
    public static final Pose launchMidRangeBlue = new Pose(58.2, 74.7, Math.toRadians(130));
    public static final Pose pickup3Pose = new Pose(49, 135, Math.toRadians(0));
    public static Path scorePreload;
    public static PathChain REDtoPPG, toPPG, drivetoPPG, drivetoPPGMid, scorePickup1, shootPGPMid, ShootPPGMid, drivetoPreload, toIntake, Intake1, ShootPickup1, toPGP, toGPPMid, toPGPMid, intakePGPGreen, intakePGPLast, shootPGP, intakePGPFirst, REDdrivetoPPG, REDscorePickup1, REDdrivetoPreload, REDtoIntake, REDIntake1, REDShootPickup1, REDtoPGP, REDintakePGPGreen, REDintakePGPLast, REDshootPGP, REDintakePGPFirst, drivetoMidRange, drivetoMidRangeRed, intakePGPGreenMid, intakePGPLastMid, intakePGPFirstMid;

    public pathsANDactions (KachowHardware robot){
        this.robot = robot;
    }
    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */



        toPGP = robot.drive.pathBuilder()
                .addPath(new BezierLine(new Pose(57.500, 17.500), new Pose(40, 57)))
                .setLinearHeadingInterpolation(Math.toRadians(113), Math.toRadians(180))
                .build();
        toPPG = robot.drive.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(57.500, 17.500),
                                new Pose(54.729, 90.168),
                                new Pose(25, 82)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(113), Math.toRadians(180))
                .build();
        intakePGPFirst = robot.drive.pathBuilder()
                .addPath(new BezierLine(new Pose(40, 57), new Pose(38, 57)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        intakePGPGreen = robot.drive.pathBuilder()
                .addPath(new BezierLine(new Pose(38, 57), new Pose(33.300, 59)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        intakePGPLast = robot.drive.pathBuilder()
                .addPath(new BezierLine(new Pose(33.300, 59), new Pose(25.600, 57)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        shootPGP = robot.drive.pathBuilder()
                .addPath(new BezierLine(new Pose(25.600, 59), new Pose(57.500, 17.500)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(113))
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
                .addPath(new BezierLine(new Pose(37.500, 35), new Pose(24.200, 34)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                //.addCallback(new DeflectorRightInPurple(robot))
                //.addPath(new BezierLine(new Pose(25.7, 35.000), new Pose()))
                //.setTangentHeadingInterpolation()
                .build();

        ShootPickup1 = robot.drive.pathBuilder()
                .addPath(new BezierLine(new Pose(24.200, 34), new Pose(57.500, 17.700)))
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
                //.addCallback(new Intaking(robot))
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */









        //MidRange
        drivetoMidRange = robot.drive.pathBuilder()
                .addPath(new BezierLine(new Pose(64.000, 8.500), launchMidRangeBlue))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(130))
                .build();
        toPGPMid = robot.drive.pathBuilder()
                .addPath(new BezierLine(launchMidRangeBlue, new Pose(40, 56)))
                .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(180))
                .build();
        intakePGPFirstMid = robot.drive.pathBuilder()
                .addPath(new BezierLine(new Pose(40, 56), new Pose(38, 56)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        intakePGPGreenMid = robot.drive.pathBuilder()
                .addPath(new BezierLine(new Pose(38, 56), new Pose(33.300, 56)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        intakePGPLastMid = robot.drive.pathBuilder()
                .addPath(new BezierLine(new Pose(33.300, 56), new Pose(25.600, 59)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        shootPGPMid = robot.drive.pathBuilder()
                .addPath(new BezierLine(new Pose(25.600, 59), launchMidRangeBlue))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))
                .build();
        drivetoPPGMid = robot.drive.pathBuilder()
                .addPath(
                        new BezierCurve(
                                launchMidRangeBlue,
                                new Pose(54.800, 32.600),
                                new Pose(37.500, 37.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(180))
                .addCallback(new Intaking(robot))
                .build();
        ShootPPGMid = robot.drive.pathBuilder()
                .addPath(new BezierLine(new Pose(24.200, 33), launchMidRangeBlue))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))
                .build();
        toGPPMid = robot.drive.pathBuilder()
                .addPath(new BezierLine(launchMidRangeBlue, new Pose(37.7, 36)))
                .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(180))
                .build();





        /// RED
        REDtoPGP = robot.drive.pathBuilder()
                .addPath(new BezierLine(new Pose(144-57.500, 17.500), new Pose(144-37.5, 61)))
                .setLinearHeadingInterpolation(Math.toRadians(68), Math.toRadians(0))
                .build();
        REDtoPPG = robot.drive.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(86.500, 17.500),
                                new Pose(87.925, 91.8),
                                new Pose(127, 83.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(68), Math.toRadians(0))
                .build();

        REDintakePGPFirst = robot.drive.pathBuilder()
                .addPath(new BezierLine(new Pose(144-37.5, 61), new Pose(144-35, 62)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        REDintakePGPGreen = robot.drive.pathBuilder()
                .addPath(new BezierLine(new Pose(144-35, 62), new Pose(144-31, 60)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        REDintakePGPLast = robot.drive.pathBuilder()
                .addPath(new BezierLine(new Pose(144-31, 60), new Pose(144-24, 62)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        REDshootPGP = robot.drive.pathBuilder()
                .addPath(new BezierLine(new Pose(144-24, 62), new Pose(144-57.500, 17.500)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(68))
                .build();

        REDdrivetoPreload = robot.drive.pathBuilder()
                .addPath(new BezierLine(new Pose(144-64.000, 8.500), new Pose(144-57.500, 17.500)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(68))
                .build();


        REDtoIntake = robot.drive.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(144-57.500, 17.500),
                                new Pose(144-54.000, 31.200),
                                new Pose(144-37.00, 35)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(68), Math.toRadians(0))
                .build();

        REDIntake1 = robot.drive.pathBuilder()
                //.addCallback(new DeflectorLeftInGreen(robot))
                .addPath(new BezierLine(new Pose(144-37.00, 37.3), new Pose(144-24.00, 37.3)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addCallback(new DeflectorRightInPurple(robot))
                //.addPath(new BezierLine(new Pose(25.7, 35.000), new Pose()))
                //.setTangentHeadingInterpolation()
                .build();

        REDShootPickup1 = robot.drive.pathBuilder()
                .addPath(new BezierLine(new Pose(144-24.00, 37.3), new Pose(144-57.500, 17.700)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(68))
                .build();

        REDdrivetoPPG = robot.drive.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(144-57.500, 17.700),
                                new Pose(144-54.000, 31.200),
                                new Pose(144-37.500, 37.3)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(68), Math.toRadians(0))
                .addCallback(new Intaking(robot))
                .build();

    }
}
