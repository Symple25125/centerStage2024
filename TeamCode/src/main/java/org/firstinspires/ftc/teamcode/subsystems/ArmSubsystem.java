package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.FeedForward;
import org.firstinspires.ftc.teamcode.util.MathUtil;

public class ArmSubsystem extends SubsystemBase {
    private final int ARM_MOTOR_COUNTS = 288;

    private final MotorEx motor;
    private final FeedForward feedForward;

    private final Telemetry telemetry;

    public ArmSubsystem(HardwareMap hMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.feedForward = new FeedForward(0.3, -33.75);

        this.motor = new MotorEx(hMap, "arm_motor");
        this.motor.resetEncoder();

        this.motor.setInverted(true);

        this.motor.setPositionCoefficient(0);
        this.motor.setPositionTolerance(MathUtil.degToCounts(1, ARM_MOTOR_COUNTS));
    }

    public void moveMotor(double power) {
        telemetry.addData("KG", String.valueOf(power));
        this.motor.setRunMode(Motor.RunMode.RawPower);
        this.motor.set(power + feedForward.calc(getCurrentPositionDeg()));
    }

    public void moveToPoint(double deg) {
        this.motor.setRunMode(Motor.RunMode.PositionControl);
        this.motor.setTargetPosition((int) MathUtil.degToCounts(deg, ARM_MOTOR_COUNTS));
        this.motor.set(1);
    }

    public boolean isAtPosition() {
        return this.motor.atTargetPosition();
    }

    public double getCurrentPositionDeg() {
        return feedForward.convertRelativeDegToAbsolute(MathUtil.countsToDeg(this.motor.getCurrentPosition(), ARM_MOTOR_COUNTS));
    }
}
