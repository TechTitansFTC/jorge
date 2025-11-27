package org.firstinspires.ftc.teamcode.util.templates;

import org.firstinspires.ftc.teamcode.util.wrappers.Hardware;

import java.util.ArrayList;

public abstract class Feature {
    public final ArrayList<Hardware> hw;

    protected Feature(ArrayList<Hardware> hw) {
        this.hw = hw;
    }

    public abstract void update();
}
