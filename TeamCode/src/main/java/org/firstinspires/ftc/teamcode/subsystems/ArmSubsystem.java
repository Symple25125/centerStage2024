package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.Motors;

@Config
public class ArmSubsystem extends SubsystemBase {
    private static ArmState currentArmState = ArmState.JOYSTICK;

    private final MotorEx right_motor;
    private final MotorEx left_motor;

    private final MotorGroup motors;

    public static final double ARM_STARTING_DEG = -43.2;
    public static final double GEAR_RATIO = Motors.RIGHT_ARM.gearRatio;
    public static final double MIN_DEG = ARM_STARTING_DEG - 35f;
    public static final double MAX_DEG = 190;

    public static double JOYSTICK_DEAD_AREA = 0.05;

    public ArmSubsystem(HardwareMap hMap, boolean shouldResetPos) {
        this.right_motor = new MotorEx(hMap, Motors.RIGHT_ARM.id);
        this.left_motor = new MotorEx(hMap, Motors.LEFT_ARM.id);

        this.left_motor.setInverted(true);

        this.motors = new MotorGroup(this.right_motor, this.left_motor);

        if(shouldResetPos) this.motors.resetEncoder();
    }

    public ArmSubsystem(HardwareMap hMap) {
        this(hMap, true);
    }

    public void moveMotor(double power) {
        this.motors.setRunMode(Motor.RunMode.RawPower);

        double newPower = this.calcPower(power);

        this.motors.set(newPower);
    }

    private double calcPower(double power) {
        if(this.getCurrentPos() >= MAX_DEG) {
            return power < 0 ? power : 0;
        }

        if(this.getCurrentPos() <= MIN_DEG) {
            return power > 0 ? power : 0;
        }

        return power;
    }

    public double getCurrentPos() {
        return (MathUtil.countsToDeg(this.right_motor.getCurrentPosition(), Motors.RIGHT_ARM.ticksPerRev) * GEAR_RATIO) + ARM_STARTING_DEG;
    }

    public boolean isAtPosition(ArmPositions pos, double safeZone) {
         double absPos = Math.abs(this.getCurrentPos() - pos.deg);

         return absPos <= safeZone;
    }

    public static void setState(ArmState state) {
        ArmSubsystem.currentArmState = state;
    }

    public static ArmState getState() {
        return currentArmState;
    }

    public static ArmPositions getNextScorePos() {
        if(currentArmState == ArmState.SCORE_LOWER) return ArmPositions.SCORE_MIDDLE;
        if(currentArmState == ArmState.SCORE_MIDDLE) return ArmPositions.SCORE_UPPER;
        return ArmPositions.SCORE_LOWER;
    }

    public enum ArmPositions {
        PICKUP(ARM_STARTING_DEG, ArmState.PICKUP),
        SCORE_LOWER(125, ArmState.SCORE_LOWER),
        SCORE_MIDDLE(105, ArmState.SCORE_MIDDLE),
        SCORE_UPPER(100, ArmState.SCORE_UPPER),
        HOOK(90, ArmState.HOOK),
        GRAB(MIN_DEG, ArmState.GRAB),
        REST(ARM_STARTING_DEG + 20, ArmState.REST),
        UNKNOWN(90, ArmState.UNKNOWN);

        public final double deg;
        public final ArmState state;

        ArmPositions(double deg) {
            this(deg, ArmState.UNKNOWN);
        }

        ArmPositions(double deg, ArmState state) {
            this.deg = deg;
            this.state = state;
        }
    }

    public enum ArmState {
        UNKNOWN,
        JOYSTICK,
        PICKUP,
        REST,
        SCORE_LOWER,
        SCORE_MIDDLE,
        SCORE_UPPER,
        HOOK,
        GRAB
    }
}
