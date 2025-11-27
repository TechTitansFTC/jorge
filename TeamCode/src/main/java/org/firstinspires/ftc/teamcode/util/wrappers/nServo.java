package org.firstinspires.ftc.teamcode.util.wrappers;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class nServo extends Hardware {

    public enum ServoType {
        AXON_MINI_V1 (Math.toRadians(60) / 0.11, 355),
        AXON_MAX_V1 (Math.toRadians(60) / 0.16, 355),
        AXON_MINI_V2 (Math.toRadians(60) / 0.11, 355),
        AXON_MAX_V2 (Math.toRadians(60) / 0.16, 355),
        AXON_MINI_SPM(Math.toRadians(60) / 0.1, 355),
        AXON_MAX_SPM(Math.toRadians(60) / 0.12, 355);

        private final double speed, rangeInRadian;

        ServoType(double speed, double rangeInRadian) {
            this.speed = speed;
            this.rangeInRadian = rangeInRadian;
        }

        public double getSpeed() {
            return speed;
        }

        public double getRangeInRadians() {
            return rangeInRadian;
        }
    }

    private final double speed, posPerRadian; // speed in units of degrees / sec
    private final Servo servo;
    private long lastLoop;
    private double currentAngle = 0, targetAngle = 0, power = 1.0, currentIntermediateTarget = 0;

    public double max = 1, min = 0;

    public nServo (String name, HardwareMap map, ServoType type) {
        this(name, map, type.getSpeed(), type.getRangeInRadians());
    }

    public nServo (String name, HardwareMap map, double speed, double rangeInRadians) {
        super(name);
        servo = map.get(Servo.class, name);
        this.speed = speed;
        this.posPerRadian = 1 / rangeInRadians;

        lastLoop = System.nanoTime();
    }

    public nServo (String name, HardwareMap map, double speed, double rangeInRadians, double max, double min) {
        this(name, map, speed, rangeInRadians);
        this.max = max;
        this.min = min;
    }

    private double convertPosToAngle(double pos) {
        return pos / posPerRadian;
    }

    private double convertAngleToPos(double ang) {
        return ang * posPerRadian;
    }

    public boolean inPosition() {
        return Math.abs(targetAngle - currentAngle) < Math.toRadians(0.01);
    }

    public boolean inPosition(double tol) {
        return Math.abs(targetAngle - currentAngle) < tol;
    }

    public void setTargetAngle (double angle) {
        setTargetAngle(angle, 1);
    }

    public void setTargetAngle (double angle, double power) {
        targetAngle = Math.min(1 * posPerRadian, Math.max(angle, 0)); // limits to 0 and 1, prolly need to setup base pos soon
        this.power = Math.max(power, 1.0);
    }

    public double getTargetAngle () {
        return targetAngle;
    }

    public void setTargetPos(double pos) {
        setTargetPos(pos, 1);
    }

    public void setTargetPos(double pos, double power) {
        setTargetAngle(convertPosToAngle(pos), power);
    }

    public double getTargetPos() {
        return convertAngleToPos(targetAngle);
    }

    public double getCurrentAngle() {
        return currentAngle;
    }
    public double getCurrentPosition() {
        return convertAngleToPos(currentAngle);
    }

    public double getMaxRadians() {
        return 1 / posPerRadian;
    }

    @Override
    public void update() {

        long currentTime = System.nanoTime();
        double timeSinceLast = ((double) currentTime - lastLoop) / 1.0E9;

        double error = targetAngle - currentAngle;
        double changeAngle = timeSinceLast * speed * power * Math.signum(error);

        currentIntermediateTarget += changeAngle;

        //straight to if change is too small or power is one
        if (Math.abs(changeAngle) > Math.abs(error) || power == 1) {
            currentIntermediateTarget = targetAngle;
        }

        if (error != 0) {
            servo.setPosition(convertAngleToPos(currentIntermediateTarget));
        }

        lastLoop = System.nanoTime();
    }

    public Servo getServo() {
        return servo;
    }
}
