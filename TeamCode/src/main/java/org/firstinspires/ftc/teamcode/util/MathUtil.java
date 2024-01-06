package org.firstinspires.ftc.teamcode.util;

public class MathUtil {
    public static double countsToDeg(int currentCounts, double maxCounts) {
        return ((currentCounts*360) / maxCounts);
    }

    public static double degToCounts(double deg, double maxCounts) {
        return ((deg*maxCounts) / 360.0);
    }
}
