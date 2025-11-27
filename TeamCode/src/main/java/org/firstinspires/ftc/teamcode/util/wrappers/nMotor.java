package org.firstinspires.ftc.teamcode.util.wrappers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class nMotor extends Hardware {
//TODO: hwq
    private final DcMotorEx motor;
    private double power, velocity;
    private boolean needsChange = true;

    public nMotor (String name, HardwareMap m) {
        super(name);
        motor = m.get(DcMotorEx.class, name);
        power = 0.0;
        velocity = 0.0;
    }

    @Override
    public void update() {
        if (needsChange) {
            motor.setPower(power);
            needsChange = false;
        }
        velocity = motor.getVelocity(); //TODO: make better for looptimes lmfao
    }

    public void setPower(double newP) {
        if (power != newP) {
            power = newP;
            needsChange = true;
        }
    }

    public double getPower() {
        return power;
    }

    public double getVelocity() {
        return velocity;
    }

    public double getCurrent(CurrentUnit unit) {
        return motor.getCurrent(unit);
    }

    public double getPosition() {
        return motor.getCurrentPosition();
    }

    public nMotor setDirection(DcMotorSimple.Direction direction) {
        motor.setDirection(direction);
        return this;
    }

    public nMotor setZPB (DcMotor.ZeroPowerBehavior zpb) {
        motor.setZeroPowerBehavior(zpb);
        return this;
    }
}
