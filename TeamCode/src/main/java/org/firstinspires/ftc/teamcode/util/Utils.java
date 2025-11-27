package org.firstinspires.ftc.teamcode.util;

public class Utils {
    public static double angleWrap (double radians) {
        radians %= (2 * Math.PI);
        if (radians > Math.PI) {
            radians -= (2 * Math.PI);
        }
        return radians;
    }

    public static double minMaxClip (double val, double min, double max) {
        return Math.max(min, Math.min(val, max));
    }
}
