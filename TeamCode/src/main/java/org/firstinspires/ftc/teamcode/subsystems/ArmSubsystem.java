package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmSubsystem extends SubsystemBase {
    private final MotorEx motor;
    private static final double FEED_FORWARD_KG = 0.5;
    private static final double STARTING_DEG = -45;

    public ArmSubsystem(HardwareMap hMap) {
        this.motor = new MotorEx(hMap, "arm_motor");
        this.motor.resetEncoder();

        this.motor.setPositionCoefficient(0);
        this.motor.setPositionTolerance(degToCounts(1));
    }

    public void moveMotor(double power) {
        this.motor.setRunMode(Motor.RunMode.RawPower);
        this.motor.set(power + ArmSubsystem.calcFeedForward(getCurrentPositionDeg()));
    }

    public void moveToPoint(double deg) {
        this.motor.setRunMode(Motor.RunMode.PositionControl);
        this.motor.setTargetPosition((int) degToCounts(deg));
        this.motor.set(1);
    }

    public boolean isAtPosition() {
        return this.motor.atTargetPosition();
    }

    public double getCurrentPositionDeg() {
        return convertRelativeToAbsolute(countsToDeg(this.motor.getCurrentPosition()));
    }

    public static double calcFeedForward(double deg) {
        return Math.cos(Math.toRadians(deg)) * FEED_FORWARD_KG;
    }

    public static double countsToDeg(int counts) {
        return ((counts*360) / 288.0);
    }

    public static double degToCounts(double deg) {
        return ((deg*288) / 360.0);
    }

    public static double convertRelativeToAbsolute(double deg) {
        return deg + STARTING_DEG;
    }
}
