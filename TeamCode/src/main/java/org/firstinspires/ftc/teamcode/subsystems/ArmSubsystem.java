package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmSubsystem extends SubsystemBase {
    private final Motor motor;
    private static final double feedForwardKG = 0.5;

    public ArmSubsystem(HardwareMap hMap) {
        this.motor = hMap.get(Motor.class, "arm_motor");
    }

    public void moveMotor(double power) {
        this.motor.set(power);
    }

    public static double calcFeedForward(double deg) {
        return Math.cos(Math.toRadians(deg)) * feedForwardKG;
    }
}
