package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.FeedForward;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.Motors;

@Config
public class ArmSubsystem extends SubsystemBase {
    private final MotorEx right_motor;
    private final MotorEx left_motor;

    private final MotorGroup motors;

    private final FeedForward feedForward;
    public static final double STARTING_DEG = -33.75;
    public static double KG = 0; // 0.27

    public static double GravitationalZeroDeg = 90;

    public ArmSubsystem(HardwareMap hMap, boolean shouldResetPos) {
        this.right_motor = new MotorEx(hMap, Motors.RIGHT_ARM.id);
        this.left_motor = new MotorEx(hMap, Motors.LEFT_ARM.id);

        this.right_motor.setInverted(true);

        this.motors = new MotorGroup(this.right_motor, this.left_motor);

        if(shouldResetPos) this.motors.resetEncoder();

        this.feedForward = new FeedForward(KG, STARTING_DEG);
    }

    public ArmSubsystem(HardwareMap hMap) {
        this(hMap, true);
    }

    public void moveMotor(double power, boolean feedForward) {
        this.motors.setRunMode(Motor.RunMode.RawPower);

        double motorPower = power + (feedForward ? this.calcCurrentFeedforward() : 0);
        this.motors.set(motorPower);
    }

    public double calcCurrentFeedforward() {
        return this.feedForward.calc(getCurrentPositionDeg(), GravitationalZeroDeg);
    }

    public void moveMotor(double power) {
        moveMotor(power, true);
    }

    public double getCurrentPositionDeg() {
        return this.feedForward.convertRelativeDegToAbsolute(MathUtil.countsToDeg(this.right_motor.getCurrentPosition(), Motors.RIGHT_ARM.ticksPerRev));
    }

    public double getSpeed() {
        return this.motors.get();
    }

    public void resetPos() {
        this.motors.resetEncoder();
    }

    public enum ArmPositions {
        TAKE(STARTING_DEG-15),
        PLACE(135),
        FOLD(STARTING_DEG-15),
        HOOK(90);

        public final double deg;

        ArmPositions(double deg) {
            this.deg = deg;
        }
    }
}
