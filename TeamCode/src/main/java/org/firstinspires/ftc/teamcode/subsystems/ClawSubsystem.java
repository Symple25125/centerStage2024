package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.Servos;

@Config
public class ClawSubsystem extends SubsystemBase {

    private final ServoEx servo;
    private static final double OFFSET = 120;

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
    }

    public void moveClawToAngle(double deg) {
        servo.turnToAngle(deg);
    }

    public double getClawDeg() {
        return servo.getAngle() + OFFSET;
    }

    public enum ClawPositions {
        OPEN(90),
        CLOSE(160),
        FULL_OPEN(45);

        public final double deg;

        ClawPositions(double deg) {
            this.deg = deg + OFFSET;
        }
    }
}
