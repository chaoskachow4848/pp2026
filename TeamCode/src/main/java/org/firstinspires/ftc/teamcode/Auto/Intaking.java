package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.paths.callbacks.PathCallback;

import org.firstinspires.ftc.teamcode.hardware.KachowHardware;

public class Intaking implements PathCallback {
    private final KachowHardware robot;

    Intaking(KachowHardware robot){
        this.robot = robot;
    }
    @Override
    public boolean run() {
        robot.intake.setPower(1);
        return true;
    }

    @Override
    public boolean isReady() {
        return true;
    }

    @Override
    public void initialize() {
        robot.intake.setPower(0);
        PathCallback.super.initialize();
    }

    @Override
    public int getPathIndex() {
        return 0;
    }


}