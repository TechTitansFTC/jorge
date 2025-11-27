package org.firstinspires.ftc.teamcode.util.wrappers;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class nCRServo extends Hardware {

    private CRServo servo;
    private AnalogInput encoder;
    private double power;
    private boolean needsChange = true;
    private double lastCheckedPosition;
    private int fullRotations;
    public double lastCheckedTime;

    private Double currentAbsoluteAngle;

    public nCRServo (String sName, String eName, HardwareMap m) {
        super(sName);
        servo = m.get(CRServo.class, sName);
        encoder = m.get(AnalogInput.class, eName);
    }

    @Override
    public void update() {
        if (needsChange) {
            servo.setPower(power);
            needsChange = false;
        }

        if (System.nanoTime() - lastCheckedTime > msToNano(200)) {
            getPosition();
        }
    }

    private double msToNano (double milliseconds) {
        return milliseconds * 1000 * 1000;
    }

    public void setPower(double newP) {
        if (newP > 1.0 || newP < 0.0) {
            throw new IllegalArgumentException("New Power cannot be greater than 1.0 or less than 0.0");
        }
        power = newP;
        needsChange = true;
    }

    public double getPower() {
        return power;
    }

    public double getPosition() {
        if (System.nanoTime() - lastCheckedTime < msToNano(50.0/3)) { //60 fps updating
            return lastCheckedPosition;
        }

        double encoderResult = (encoder.getVoltage() / 3.3) * 360;

        if (lastCheckedPosition >= 180 && encoderResult <= 180 && Math.signum(power) > 0) {
            fullRotations++;
        } else if (lastCheckedPosition <= 180 && encoderResult >= 180 && Math.signum(power) < 0) {
            fullRotations--;
        }

        if (currentAbsoluteAngle != null) {
            currentAbsoluteAngle = encoderResult + fullRotations * 360;
        }

        lastCheckedPosition = encoderResult;
        lastCheckedTime = System.nanoTime();
        return (fullRotations * 360) + lastCheckedPosition;
    }

    public void setCurrentAbsoluteAngle(double angle) {
        this.currentAbsoluteAngle = angle;
    }

    public double getCurrentAbsoluteAngle() {
        return this.currentAbsoluteAngle;
    }

    public double getLastCheckedPosition() {
        return lastCheckedPosition;
    }

    public double getLastCheckedTime() {
        return lastCheckedTime;
    }
}
