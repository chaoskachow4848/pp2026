package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.paths.callbacks.PathCallback;

import org.firstinspires.ftc.teamcode.hardware.KachowHardware;

public class DeflectorRightInPurple implements PathCallback {
    private final KachowHardware robot;

    DeflectorRightInPurple(KachowHardware robot){
        this.robot = robot;
    }
    @Override
    public boolean run() {
        robot.deflector.setPosition(.6683);
        return true;
    }

    @Override
    public boolean isReady() {
        return true;
    }

    @Override
    public void initialize() {
        robot.deflector.setPosition(.6683);
        PathCallback.super.initialize();
    }

    @Override
    public int getPathIndex() {
        return 0;
    }


}