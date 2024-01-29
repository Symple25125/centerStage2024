package org.firstinspires.ftc.teamcode.util;

public enum Motors {
    ARM("arm_motor"),
    CLAW_MOTOR("claw_motor"),
    LEFT_LEG("left_leg"),
    RIGHT_LEG("right_leg");

    public final String id;

    Motors(String id) {
        this.id = id;
    }
}
