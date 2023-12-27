package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmSubsystem extends SubsystemBase {
    private final Motor motor;
    private static final double FEED_FORWARD_KG = 0.5;
    private static final double STARTING_DEG = -45;

    public ArmSubsystem(HardwareMap hMap) {
        this.motor = hMap.get(Motor.class, "arm_motor");
        this.motor.resetEncoder();
    }

    public void moveMotor(double power) {
        this.motor.set(power + ArmSubsystem.calcFeedForward(getCurrentPositionDeg()));
    }

    public double getCurrentPositionDeg() {
        return convertRelativeToAbsolute(countsToDeg(this.motor.getCurrentPosition()));
    }

    public static double calcFeedForward(double deg) {
        return Math.cos(Math.toRadians(deg)) * FEED_FORWARD_KG;
    }

    public static double countsToDeg(int counts) {
        return  ((counts*360) / 288.0);
    }

    public static double convertRelativeToAbsolute(double deg) {
        return deg + STARTING_DEG;
    }
}
