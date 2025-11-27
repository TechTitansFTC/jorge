package org.firstinspires.ftc.teamcode.subsystems.Outtake;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.templates.Feature;
import org.firstinspires.ftc.teamcode.util.wrappers.nServo;

import java.util.ArrayList;

public class Hood extends Feature {

    private nServo left, right;
    // arc radius is 157 mm

    public Hood (HardwareMap map) {
        super(new ArrayList<>());
        
        left = new nServo("hl", map, nServo.ServoType.AXON_MINI_V2);
        right = new nServo("hr", map, nServo.ServoType.AXON_MINI_V2);

        left.getServo().setPosition(1);
        right.getServo().setPosition(1);

        hw.add(left);
        hw.add(right);
    }

    public void update() {
        left.update();
        right.update();
    }

    private double AngleToPosition(double angle) {
        return 0; // do some magic maths
    }

    private double PositionToAngle(double position) {
        return 0; // do some magic inverse maths
    }

    public boolean atAngle (double angle) {
        return false; // magic behind the scenes convert to position and check position
    }

    public boolean atPosition (double position) {
        return left.inPosition() && right.inPosition();
    }

    public void setAngle (double angle) {
        // use angle to position and then set position
    }

    public void setPosition (double position) {
        left.getServo().setPosition(position);
        right.getServo().setPosition(position);
    }
}
