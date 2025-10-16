package org.firstinspires.ftc.teamcode.hardware;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;


public class kaze {
    public volatile static Pose robotPose;
    public volatile static double headingOffset;
    public volatile static boolean isActionDone = false;

    public static void init(Pose startPose){
        robotPose = startPose;
        headingOffset = Math.toDegrees(startPose.getHeading());
    }

    public static void update(Follower drive){
        if(robotPose == null){
            robotPose = new Pose(60,60,180);
        }
        drive.updatePose();
        robotPose = drive.getPose();

        drawCurrentAndHistory(drive);

    }

    public static void drawCurrent(Follower drive) {
        try {
            Drawing.drawRobot(drive.getPose());
            Drawing.sendPacket();
        } catch (Exception e) {
            throw new RuntimeException("Drawing failed " + e);
        }
    }

    public static void drawCurrentAndHistory(Follower drive) {
        Drawing.drawPoseHistory(drive.getPoseHistory());
        drawCurrent(drive);
    }
}
