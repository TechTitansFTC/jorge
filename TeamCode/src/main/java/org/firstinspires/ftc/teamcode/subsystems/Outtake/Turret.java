package org.firstinspires.ftc.teamcode.subsystems.Outtake;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.PIDF;
import org.firstinspires.ftc.teamcode.util.Utils;
import org.firstinspires.ftc.teamcode.util.templates.Feature;
import org.firstinspires.ftc.teamcode.util.wrappers.nCRServo;

import java.util.ArrayList;

public class Turret extends Feature {

    private static final double minTargetAngle = -135, maxTargetAngle = 135, tolerance = 3, minPower = 0.05;
    private double currentAngle, targetAngle;
    private PIDF turretPID = new PIDF(0, 0);
    private final nCRServo turretServo;

    public Turret (HardwareMap map) {
        super(new ArrayList<>());

        turretServo = new nCRServo("turret", "turretEncoder", map);
        currentAngle = 0;
        targetAngle = 0;
        turretServo.setCurrentAbsoluteAngle(currentAngle);
        turretPID.setTolerance(tolerance);

        hw.add(turretServo);
    }

    public void update() {
        turretServo.update();
        currentAngle = turretServo.getCurrentAbsoluteAngle();

        double pow = 0;

        if (!inPosition()) {
            pow = turretPID.calculate(currentAngle);
            if (Math.abs(pow) < minPower) pow = minPower * Math.signum(pow);
        }

        turretServo.setPower(pow);
    }

    public void setTargetAngle(double angle) {
        targetAngle = Utils.minMaxClip(angle, minTargetAngle, maxTargetAngle);
        turretPID.setSetPoint(targetAngle);
    }

    public void setTargetAngleRadians(double angle) {
        setTargetAngle(Math.toDegrees(angle));
    }

    public boolean inPosition() {
        return turretPID.atSetPoint();
    }

    public double getCurrentTarget() {
        return targetAngle;
    }

    public double getCurrentAngle() {
        return currentAngle;
    }
}
