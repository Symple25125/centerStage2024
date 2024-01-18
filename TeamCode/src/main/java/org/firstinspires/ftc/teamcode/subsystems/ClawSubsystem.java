package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ClawSubsystem extends SubsystemBase {

    private final ServoEx servo;
    private static final double OFFSET = 285;

    public ClawSubsystem(HardwareMap hMap) {
        this.servo = new SimpleServo(hMap, "claw_servo", 0, 360, AngleUnit.DEGREES);
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

    public enum ClawPositions {
        OPEN(-45),
        CLOSE(0);

        public final double deg;

        ClawPositions(double deg) {
            this.deg = deg + OFFSET;
        }
    }
}
