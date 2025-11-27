package org.firstinspires.ftc.teamcode.ShooterProfiling;

import java.util.NavigableMap;
import java.util.TreeMap;

public class InterpLUT {

    private final NavigableMap<Double, ShooterPair> table = new TreeMap<>();

    public void addPoint(double distance, double vel, double hood) {
        addPoint(distance, new ShooterPair(vel, hood));
    }

    public void addPoint(double distance, ShooterPair settings) {
        table.put(distance, settings);
    }

    public ShooterPair lookup (double distance) {
        if (table.containsKey(distance)) {
            return table.get(distance);
        }

        NavigableMap.Entry<Double, ShooterPair> lower = table.lowerEntry(distance);
        NavigableMap.Entry<Double, ShooterPair> higher = table.higherEntry(distance);

        if (lower == null || higher == null) {
            return null;//98.45313
        }

        double x1 = lower.getKey();
        ShooterPair p1 = lower.getValue();
        double x2 = higher.getKey();
        ShooterPair p2 = higher.getValue();

        double vel = interpolate(distance, x1, x2, p1.getVel(), p2.getVel());
        double hood = interpolate(distance, x1, x2, p1.getHood(), p2.getHood());

        return new ShooterPair(vel, hood);
    }

    private double interpolate (double x, double x1, double x2, double y1, double y2) {
        return (y1 + ((x - x1) * (y2 - y1) / (x2 - x1)));
    }
}
