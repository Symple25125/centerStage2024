package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.FeedForward;

public class JointSubSystem extends SubsystemBase {
    private final ServoEx servo;

    public JointSubSystem(HardwareMap hMap) {
        this.servo = new SimpleServo(hMap, "claw_joint_servo",0, 360, AngleUnit.DEGREES);
    }

    public void moveServo(double position) {
        servo.setPosition(position);
    }

    // in deg
    public enum JointPositions {
        START(0),
        PICKUP(0),
        PUT(0);


        public final double deg;

        JointPositions(double deg) {
            this.deg = deg;
        }
    }
}
