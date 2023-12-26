package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveBaseSubsystem extends SubsystemBase {
    private final Motor leftMotor, rightMotor;
    public DriveBaseSubsystem(final HardwareMap hMap) {
        this.leftMotor = hMap.get(Motor.class, "left_leg");
        this.rightMotor = hMap.get(Motor.class, "right_leg");
    }

    public void moveMotors(float leftPower, float rightPower) {
        leftMotor.set(leftPower);
        rightMotor.set(rightPower);
    }
}