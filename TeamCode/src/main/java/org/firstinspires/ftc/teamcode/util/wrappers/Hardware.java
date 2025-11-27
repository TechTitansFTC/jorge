package org.firstinspires.ftc.teamcode.util.wrappers;

public abstract class Hardware {

    //TODO: priority, calcs, hwq
//    protected final double basePriority, priorityScale;
    public final String name;
//    protected double lastUpdateTime, callLengthMillis;
    boolean isUpdated = false;

    public Hardware ( String name) { //double basePriority, double priorityScale,
//        this.basePriority = basePriority;
//        this.priorityScale = priorityScale;
        this.name = name;
//        lastUpdateTime = System.nanoTime();
    }

//    protected abstract double getPriority (double timeRemaining);

    protected abstract void update();

    public void resetUpdateBoolean() {
        isUpdated = false;
    }

    public String getName() {
        return name;
    }
}
