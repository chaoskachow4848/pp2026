package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.paths.callbacks.PathCallback;

import org.firstinspires.ftc.teamcode.hardware.KachowHardware;

public class DeflectorLeftInGreen implements PathCallback {
    private final KachowHardware robot;

    DeflectorLeftInGreen(KachowHardware robot){
        this.robot = robot;
    }
    @Override
    public boolean run() {
        robot.deflector.setPosition(.3583);
        return true;
    }

    @Override
    public boolean isReady() {
        return true;
    }

    @Override
    public void initialize() {
        robot.deflector.setPosition(.3583);
        PathCallback.super.initialize();
    }

    @Override
    public int getPathIndex() {
        return 0;
    }


}