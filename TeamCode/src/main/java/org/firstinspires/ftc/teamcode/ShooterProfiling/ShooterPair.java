package org.firstinspires.ftc.teamcode.ShooterProfiling;

public class ShooterPair {
    private double vel, hood;

    public ShooterPair () {
        this(0, 0);
    }

    public ShooterPair (double vel, double hood) {
        this.vel = vel;
        this.hood = hood;
    }

    public ShooterPair setVel(double vel) {
        this.vel = vel;
        return this;
    }

    public ShooterPair setHood(double hood) {
        this.hood = hood;
        return this;
    }

    public double getVel() {
        return vel;
    }

    public double getHood() {
        return hood;
    }
}
