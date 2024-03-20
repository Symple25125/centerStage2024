package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.Motors;

public class DriveBaseSubsystem extends SubsystemBase {

//    public static final double MAX_RPM = Motors.RIGHT_LEG.maxRPM;
    public static final double TICKS_PER_REV = Motors.RIGHT_LEG.ticksPerRev;
    public static final double GEAR_RATIO = Motors.RIGHT_LEG.gearRatio;
    public static final double WHEEL_RADIUS = 0.045f;
    public static final double WHEEL_DIAMETER = 0.37f;
    public static final double METERS_PER_REV = (Math.PI * 2) * WHEEL_RADIUS;
    public static final double METERS_PER_TICK = (METERS_PER_REV / (TICKS_PER_REV * GEAR_RATIO));
    public static final double SLOW_SPEED_MODIFIER = 0.5;
    public static final double NORMAL_SPEED_MODIFIER = 1;
    public static final double Ks = 0.07f;
    public static final RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIRECTION = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public static final RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIRECTION = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
    public static double LINER_SPEED_MODIFIER = 1;
    public static double TURN_SPEED_MODIFIER = 1;
    public static boolean INVERT = false;


    private final MotorEx leftMotor, rightMotor;
    private final BHI260IMU imu;
    public DriveBaseSubsystem(final HardwareMap hMap) {
        this.rightMotor = new MotorEx(hMap, Motors.RIGHT_LEG.id);
        this.leftMotor = new MotorEx(hMap,Motors.LEFT_LEG.id);
        this.rightMotor.setInverted(true);


        BHI260IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(LOGO_FACING_DIRECTION, USB_FACING_DIRECTION));

        this.imu = hMap.get(BHI260IMU.class, "imu");

        this.imu.initialize(parameters);
    }

    public void moveMotors(double leftPower, double rightPower) {
        leftMotor.set(leftPower);
        rightMotor.set(rightPower);
    }

    private double getMotorLeftEncoderPos() {
        return this.leftMotor.getCurrentPosition();
    }
    private double getMotorRightEncoderPos() {
        return this.rightMotor.getCurrentPosition();
    }

    public double getLeftWheelDistanceDriven() {
        double tick = this.getMotorLeftEncoderPos();
        return this.encoderTicksToMeter(tick);
    }
    public double getRightWheelDistanceDriven() {
        double tick = this.getMotorRightEncoderPos();
        return this.encoderTicksToMeter(tick);
    }

    public double encoderTicksToMeter(double ticks) {
        return ticks * METERS_PER_TICK;
    }

    public double getHeadingByWheels() {
       double right = this.getRightWheelDistanceDriven();
       double left = this.getLeftWheelDistanceDriven();
       return Math.toDegrees((right - left) / WHEEL_DIAMETER);
    }

    public double getHeadingByGyro() {
        return this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
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