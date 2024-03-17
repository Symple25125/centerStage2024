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
    private final MotorEx right_motor;
    private final MotorEx left_motor;

    private final MotorGroup motors;

    public static final double ARM_STARTING_DEG = -49f;
    public static final double GEAR_RATIO = Motors.RIGHT_ARM.gearRatio;
    public static final double MIN_DEG = ARM_STARTING_DEG - 75f;
    public static final double MAX_DEG = 190f;

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

    public enum ArmPositions {
        TAKE(ARM_STARTING_DEG),
        PLACE(110),
        FOLD(ARM_STARTING_DEG),
        HOOK(90),
        GRAB(MIN_DEG),
        REST(ARM_STARTING_DEG+35);

        public final double deg;

        ArmPositions(double deg) {
            this.deg = deg;
        }
    }
}
