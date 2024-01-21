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
    private static final double OFFSET = 0;
    private static final double GEAR_RATIO = 90f / 72f;

    public JointSubSystem(HardwareMap hMap) {
        this.servo = new SimpleServo(hMap, "claw_joint_servo",0, 360, AngleUnit.DEGREES);
    }

    public void moveServo(JointPositions position) {
        servo.turnToAngle(position.deg);
    }

    public double getClawJointAngle() {
        return servo.getAngle(AngleUnit.DEGREES) + OFFSET;
    }

    // in deg
    public enum JointPositions {
        PICKUP(220),
        PUT(145);


        public final double deg;

        JointPositions(double deg) {
            this.deg = (deg + OFFSET) * GEAR_RATIO;
        }
    }
}
