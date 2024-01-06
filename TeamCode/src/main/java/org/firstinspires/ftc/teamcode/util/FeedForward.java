package org.firstinspires.ftc.teamcode.util;

public class FeedForward {
    private final double KG;
    private final double STARTING_DEG;

    public FeedForward(double kg) {
        this(kg, 0);
    }

    public FeedForward(double kg, double startingDeg) {
        this.KG = kg;
        this.STARTING_DEG = startingDeg;
    }

    /**
     * @param deg the current object deg with the starting pos
     * @return the power to add for the object to stay up
     */
    public double calc(double deg) {
        return Math.cos(Math.toRadians(deg)) * KG;
    }

    /**
     * @param deg current deg
     * @return current deg + starting deg
     */
    public double convertRelativeDegToAbsolute(double deg) {
        return deg + STARTING_DEG;
    }
}
