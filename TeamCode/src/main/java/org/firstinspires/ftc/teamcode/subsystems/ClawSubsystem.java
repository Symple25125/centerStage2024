package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawSubsystem extends SubsystemBase {

    private final Servo servo;

    public ClawSubsystem(HardwareMap hMap) {
        this.servo = hMap.get(Servo.class, "claw_servo");
    }

    public void openClaw() {
        servo.setPosition(ClawPositions.OPEN.deg);
    }

    public void closeClaw() {
        servo.setPosition(ClawPositions.CLOSE.deg);
    }

    public enum ClawPositions {
        OPEN(0),
        CLOSE(0);

        public final double deg;

        ClawPositions(double deg) {
            this.deg = deg;
        }
    }
}
