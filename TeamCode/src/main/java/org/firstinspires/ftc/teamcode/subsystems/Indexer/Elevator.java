package org.firstinspires.ftc.teamcode.subsystems.Indexer;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.util.templates.Feature;
import org.firstinspires.ftc.teamcode.util.wrappers.nServo;

import java.util.ArrayList;

public class Elevator extends Feature {


    private nServo left, right;
    private Constants.ElevatorPosition status;

    public Elevator (HardwareMap map) {
        super(new ArrayList<>());

        left = new nServo("el", map, nServo.ServoType.AXON_MAX_V2);
        right = new nServo("er", map, nServo.ServoType.AXON_MAX_V2);
        status = Constants.ElevatorPosition.UP;

        hw.add(left);
        hw.add(right);
    }

    public void update() {
        left.getServo().setPosition(status.getLeft());
        right.getServo().setPosition(status.getRight());

        left.update();
        right.update();
    }

    public void setStatus(Constants.ElevatorPosition position) {
        this.status = position;
    }

    public Constants.ElevatorPosition getStatus() {
        return status;
    }

    public boolean inPosition() {
        return left.inPosition() && right.inPosition();
    }
}
