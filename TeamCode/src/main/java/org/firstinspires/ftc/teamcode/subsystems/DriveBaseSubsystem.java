package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Motors;

import java.util.ArrayList;
import java.util.HashMap;

public class DriveBaseSubsystem extends SubsystemBase {

//    public static final double MAX_RPM = Motors.RIGHT_LEG.maxRPM;
//    public static final double TICK_PER_REV = Motors.RIGHT_LEG.ticksPerRev;
//    public static final double GEAR_RATIO = Motors.RIGHT_LEG.gearRatio;
//    public static final double WHEEL_RADIUS = 0.045;
//    public static final double METERS_PER_REV = (Math.PI * 2) * WHEEL_RADIUS;
//    public static final double METERS_PER_TICK = (METERS_PER_REV / (TICK_PER_REV * GEAR_RATIO));
    public static final double SLOW_SPEED_MODIFIER = 0.5;
    public static final double NORMAL_SPEED_MODIFIER = 1;
    public static double LINER_SPEED_MODIFIER = 1;
    public static double TURN_SPEED_MODIFIER = 1;
    public static boolean INVERT = false;


    private final MotorEx leftMotor, rightMotor;
    public DriveBaseSubsystem(final HardwareMap hMap) {
        this.rightMotor = new MotorEx(hMap, Motors.RIGHT_LEG.id);
        this.leftMotor = new MotorEx(hMap,Motors.LEFT_LEG.id);
        this.rightMotor.setInverted(true);
    }

    public void moveMotors(double leftPower, double rightPower) {
        leftMotor.set(leftPower);
        rightMotor.set(rightPower);
    }

    public void moveWithJoyStickAndNormalize(GamepadEx controller) {
        double rotationSpeed = controller.getRightX() * 0.5f * TURN_SPEED_MODIFIER * (INVERT ? -1 : 1);
        double linerSpeed = controller.getLeftY() * 0.8f * LINER_SPEED_MODIFIER * (INVERT ? -1 : 1);

        double rawLeftSpeed = (linerSpeed + rotationSpeed);
        double rawRightSpeed = (linerSpeed - rotationSpeed);

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

    public void changeSpeedModifier(double modifier) {
        changeSpeedModifier(modifier, modifier);
    }

    public void changeSpeedModifier(double linerModifier, double turnModifier) {
        LINER_SPEED_MODIFIER = linerModifier;
        TURN_SPEED_MODIFIER = turnModifier;
    }

    public void setInverted(boolean invert) {
        INVERT = invert;
    }
}