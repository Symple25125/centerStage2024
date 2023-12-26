package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawSubsystem extends SubsystemBase {

    private final Servo servo;
    private final double OPEN_ROTATION = 0;
    private final double CLOSE_ROTATION = 0;

    public ClawSubsystem(HardwareMap hMap) {
        this.servo = hMap.get(Servo.class, "claw_servo");
    }

    public void openClaw() {
        servo.setPosition(OPEN_ROTATION);
    }

    public void closeClaw() {
        servo.setPosition(CLOSE_ROTATION);
    }

}
