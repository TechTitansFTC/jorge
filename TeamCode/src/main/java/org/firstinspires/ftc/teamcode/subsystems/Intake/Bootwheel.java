package org.firstinspires.ftc.teamcode.subsystems.Intake;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.templates.Feature;
import org.firstinspires.ftc.teamcode.util.wrappers.nMotor;

import java.util.ArrayList;

public class Bootwheel extends Feature {
    private nMotor leftMotor, rightMotor;

    public Bootwheel (HardwareMap map) {
        super(new ArrayList<>());

        leftMotor = new nMotor("li", map);
        rightMotor = new nMotor("ri", map);

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        hw.add(leftMotor);
        hw.add(rightMotor);
    }

    public void update() {
        leftMotor.update();
        rightMotor.update();
    }

    public void setPower(double power) {
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }

    public double getPower() {
        return leftMotor.getPower();
    }
}
