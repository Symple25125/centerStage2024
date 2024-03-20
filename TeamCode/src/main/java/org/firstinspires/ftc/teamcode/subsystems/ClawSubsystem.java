package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.Servos;

@Config
public class ClawSubsystem extends SubsystemBase {
    private static ClawState currentState;

    private final ServoEx servo;
    private static final double OFFSET = 190;

    public ClawSubsystem(HardwareMap hMap) {
        this.servo = new SimpleServo(hMap, Servos.CLAW.id, 0, 360, AngleUnit.DEGREES);
    }

    public void openClaw() {
        moveClawToPosition(ClawPositions.OPEN);
    }

    public void closeClaw() {
        moveClawToPosition(ClawPositions.CLOSE);
    }

    public void moveClawToPosition(ClawPositions clawPositions) {
        servo.turnToAngle(clawPositions.deg);
        ClawSubsystem.setState(clawPositions.state);
    }

    public void moveClawToAngle(double deg) {
        servo.turnToAngle(deg);
    }

    public double getClawDeg() {
        return servo.getAngle() + OFFSET;
    }

    public static void setState(ClawState currentState) {
        ClawSubsystem.currentState = currentState;
    }

    public static ClawState getCurrentState() {
        return currentState;
    }

    public enum ClawPositions {
        OPEN(90, ClawState.OPEN),
        CLOSE(160, ClawState.CLOSE);

        public final double deg;
        public final ClawState state;

        ClawPositions(double deg) {
            this(deg, ClawState.UNKNOWN);
        }

        ClawPositions(double deg, ClawState state) {
            this.deg = deg + OFFSET;
            this.state = state;
        }
    }

    public enum ClawState {
        UNKNOWN,
        OPEN,
        CLOSE
    }
}
