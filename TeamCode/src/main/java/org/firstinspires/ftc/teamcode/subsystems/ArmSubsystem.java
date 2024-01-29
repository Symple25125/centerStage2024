package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.FeedForward;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.Motors;

@Config
public class ArmSubsystem extends SubsystemBase {
    private final int ARM_MOTOR_COUNTS = 288;

    private final MotorEx motor;
    private final JointSubSystem jointSubSystem;
    private final FeedForward feedForward;
    private static final double STARTING_DEG = -33.75;
    public static double KGClawJoint = 0.2;
    public static double KG = 0.3;

    public ArmSubsystem(HardwareMap hMap, JointSubSystem jointSubSystem) {
        this.feedForward = new FeedForward(KG, STARTING_DEG);
        this.jointSubSystem = jointSubSystem;

        this.motor = new MotorEx(hMap, Motors.ARM.id);
        this.motor.resetEncoder();

        this.motor.setInverted(true);
    }

    public void moveMotor(double power, boolean feedForward) {
        this.motor.setRunMode(Motor.RunMode.RawPower);

        double motorPower = power + (feedForward ? this.calcCurrentFeedforward() : 0);
        this.motor.set(motorPower);
    }

    private double calcCurrentFeedforward() {
        double clawJointDeg = this.jointSubSystem.getClawJointAngle();
        double calcClawJointPower = Math.abs(Math.cos(Math.toRadians(clawJointDeg))) * KGClawJoint * -1;

        return this.feedForward.calc(getCurrentPositionDeg()) + (getCurrentPositionDeg() > 90 ? calcClawJointPower : 0);
//        return this.feedForward.calc(getCurrentPositionDeg());
    }

    public void moveMotor(double power) {
        moveMotor(power, true);
    }

    public double getCurrentPositionDeg() {
        return this.feedForward.convertRelativeDegToAbsolute(MathUtil.countsToDeg(this.motor.getCurrentPosition(), ARM_MOTOR_COUNTS));
    }

    public enum ArmPositions {
        TAKE(STARTING_DEG-15),
        PLACE(110),

        UNDER_DRIVE_DOWN(-15),
        UNDER_DRIVE_UP(55);

        public final double deg;

        ArmPositions(double deg) {
            this.deg = deg;
        }
    }
}
