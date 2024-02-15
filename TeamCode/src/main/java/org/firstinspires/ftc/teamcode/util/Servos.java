package org.firstinspires.ftc.teamcode.util;

public enum Servos {
    CLAW_JOINT("claw_joint_servo"),
    CLAW("claw_servo"),
    DRONE("drone_servo");
    public final String id;

    Servos(String id) {
        this.id = id;
    }
}
