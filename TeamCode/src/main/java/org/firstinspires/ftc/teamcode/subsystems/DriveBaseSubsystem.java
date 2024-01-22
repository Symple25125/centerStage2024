package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Motors;

public class DriveBaseSubsystem extends SubsystemBase {
    private final MotorEx leftMotor, rightMotor;
    public DriveBaseSubsystem(final HardwareMap hMap) {
        this.rightMotor = new MotorEx(hMap, Motors.RIGHT_LEG.id);
        this.leftMotor = new MotorEx(hMap,Motors.LEFT_LEG.id);
        this.leftMotor.setInverted(true);
    }

    public void moveMotors(double leftPower, double rightPower) {
        leftMotor.set(leftPower);
        rightMotor.set(rightPower);
    }

    public void moveWithJoyStickAndNormalize(GamepadEx controller, double linerModifier, double turnModifier) {
        double rotationSpeed = controller.getRightX() * 0.5f * turnModifier;
        double linerSpeed = -controller.getLeftY() * 0.8f * linerModifier;

        double rawLeftSpeed = (linerSpeed - rotationSpeed);
        double rawRightSpeed = (linerSpeed + rotationSpeed);

        double normalizedLeftSpeed = rawLeftSpeed;
        double normalizedRightSpeed = rawRightSpeed;


        double absRightSpeed = Math.abs(rawRightSpeed);
        double absLeftSpeed = Math.abs(rawLeftSpeed);
        double maxValue = Math.max(absLeftSpeed, absRightSpeed);

        if(maxValue >= 1) {
            normalizedLeftSpeed = rawLeftSpeed / maxValue;
            normalizedRightSpeed = rawRightSpeed / maxValue;
        }

        this.moveMotors(normalizedLeftSpeed, normalizedRightSpeed);
    }
}