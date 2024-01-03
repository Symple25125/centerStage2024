package org.firstinspires.ftc.teamcode.util;

public class FeedForward {
    private final double FEED_FORWARD_KG;
    private final double STARTING_DEG;

    public FeedForward(double kg) {
        this(kg, 0);
    }

    public FeedForward(double kg, double startingDeg) {
        this.FEED_FORWARD_KG = kg;
        this.STARTING_DEG = startingDeg;
    }

    public double calcFeedForward(double deg) {
        return Math.cos(Math.toRadians(deg)) * FEED_FORWARD_KG;
    }

    public double countsToDeg(int counts) {
        return ((counts*360) / 288.0);
    }

    public double degToCounts(double deg) {
        return ((deg*288) / 360.0);
    }

    public double convertRelativeToAbsolute(double deg) {
        return deg + STARTING_DEG;
    }
}
