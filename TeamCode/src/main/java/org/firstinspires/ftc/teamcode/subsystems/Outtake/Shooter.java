package org.firstinspires.ftc.teamcode.subsystems.Outtake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.util.PIDF;
import org.firstinspires.ftc.teamcode.util.templates.Feature;
import org.firstinspires.ftc.teamcode.util.wrappers.nMotor;

import java.util.ArrayList;

@Config
public class Shooter extends Feature {

    private final static double tolerance = 40;
    public nMotor leftShooter, rightShooter;
    private VoltageSensor voltage;
    public static double P = 0.07, D = 0.05, F = 0.02;
    private PIDF velPID = new PIDF(P,D,F);
    private double currentVel, targetVel, pow;

    public Shooter (HardwareMap map) {
        super(new ArrayList<>());

        leftShooter = new nMotor("sl", map);
        rightShooter = new nMotor("sr", map);

        voltage = map.get(VoltageSensor.class, "Control Hub");

        velPID.setTolerance(tolerance);

        hw.add(leftShooter);
        hw.add(rightShooter);
    }

    public void update() {
        leftShooter.update();
        rightShooter.update();

        velPID.setP(P);
        velPID.setD(D);
        velPID.setF(F);

        currentVel = leftShooter.getVelocity();
        if (!atVelocity()) {
            pow = velPID.calculate(currentVel, targetVel);
        } else {
            pow = F * targetVel;
        }
        pow /= voltage.getVoltage();

        leftShooter.setPower(pow);
        rightShooter.setPower(pow);
    }

    public void setTargetVel (double vel) {
        targetVel = Math.max(vel, 0);
        velPID.setSetPoint(targetVel);
    }

    public boolean atVelocity () {
        velPID.calculate(currentVel);
        return velPID.atSetPoint();
    }

    public double getCurrentVel() {
        return currentVel;
    }

    public double getTargetVel() {
        return targetVel;
    }

    @Override
    public String toString() {
        return "Shooter{" +
                "velPID=" + velPID.getPositionalError() +
                ", currentVel=" + currentVel +
                ", targetVel=" + targetVel +
                ", setPower=" + velPID.calculate(currentVel) +
                ", voltage=" + voltage.getVoltage() +
                ", pow=" + pow +
                '}';
    }
}
