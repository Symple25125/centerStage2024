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

    public static final double MAX_RPM = 6000;
    public static final double TICK_PER_REV = 28;
    public static final double GEAR_RATIO = 20/1;
    public static final double WHEEL_RADIUS = 0.045;
    public static final double METERS_PER_REV = (Math.PI * 2) * WHEEL_RADIUS;
    public static final double METERS_PER_TICK = (METERS_PER_REV / (TICK_PER_REV * GEAR_RATIO));

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

    public void moveWithJoyStickAndNormalize(GamepadEx controller, double linerModifier, double turnModifier) {
        double rotationSpeed = controller.getRightX() * 0.5f * turnModifier;
        double linerSpeed = controller.getLeftY() * 0.8f * linerModifier;

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

    public HashMap<String, Double> getMotorsVelocity() {
        HashMap<String, Double> vels = new HashMap<>();

        double leftVel = this.leftMotor.getVelocity() * METERS_PER_TICK;
        double rightVel = this.rightMotor.getVelocity() * METERS_PER_TICK;

        vels.put("left", leftVel);
        vels.put("right", rightVel);
        return vels;
    }

    public HashMap<String, Double> getMotorsAcceleration(MultipleTelemetry telemetry) {
        HashMap<String, Double> acc = new HashMap<>();
        
        double leftAcc = this.leftMotor.getAcceleration() * METERS_PER_TICK;
        double rightAcc = this.rightMotor.getAcceleration() * METERS_PER_TICK;
        telemetry.addData("leftAcc", leftAcc);
        telemetry.addData("rightAcc", rightAcc);
        acc.put("left", leftAcc);
        acc.put("right", rightAcc);

        return acc;
    }
}