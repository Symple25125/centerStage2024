package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.FeedForward;

public class JointSubSystem extends SubsystemBase {
    private final ServoEx servo;

    public JointSubSystem(HardwareMap hMap) {
        this.servo = hMap.get(ServoEx.class, "claw_joint_servo");
    }

    public void moveServo(double position) {
        servo.setPosition(position);
    }

    public static class JointPositions {
        public static final double PICKUP = 0;
        public static final double PUT = 0;
    }
}
