package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.FeedForward;
import org.firstinspires.ftc.teamcode.util.Servos;

@Config
public class JointSubSystem extends SubsystemBase {
    private final ServoEx servo;
    private static final double GEAR_RATIO = 90f / 72f;
    private static final double OFFSET = 0;

    public JointSubSystem(HardwareMap hMap) {
        this.servo = new SimpleServo(hMap, Servos.CLAW_JOINT.id, 0, 360, AngleUnit.DEGREES);
    }

    public void moveServo(JointPositions position) {
        servo.turnToAngle(position.deg);
    }

    public double getClawJointAngle() {
        return servo.getAngle(AngleUnit.DEGREES);
    }

    // in deg
    public enum JointPositions {
        PICKUP(65),
        PUT(0),
        HOOK(280);


        public final double deg;

        JointPositions(double deg) {
            this.deg = (deg * GEAR_RATIO) - OFFSET;
        }
    }
}
