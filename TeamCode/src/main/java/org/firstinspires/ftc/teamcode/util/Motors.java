package org.firstinspires.ftc.teamcode.util;

public enum Motors {
    RIGHT_ARM("arm_motor_one", 125, 288, 30/125f),
    LEFT_ARM("arm_motor_two", 125, 288, 30/125f),
    LEFT_LEG("left_leg", 6000, 28, 20/1f),
    RIGHT_LEG("right_leg", 6000, 28, 20/1f);

    public final String id;
    public final double maxRPM;
    public final int ticksPerRev;
    public final double gearRatio;

    Motors(String id, double maxRPM, int ticksPerRev, double gearRatio) {
        this.id = id;
        this.maxRPM = maxRPM;
        this.ticksPerRev = ticksPerRev;
        this.gearRatio = gearRatio;
    }

    Motors(String id, double maxRPM, int ticksPerRev) {
        this(id, maxRPM, ticksPerRev, 1);
    }
}
