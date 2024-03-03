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
    public static final double OFFSET = 155;

    public JointSubSystem(HardwareMap hMap) {
        this.servo = new SimpleServo(hMap, Servos.CLAW_JOINT.id, OFFSET - 180, OFFSET + 180, AngleUnit.DEGREES);
    }

    public void moveServo(JointPositions position) {
        servo.turnToAngle(position.deg);
    }

    public double getClawJointAngle() {
        return servo.getAngle(AngleUnit.DEGREES);
    }

    public enum JointPositions {
        PICKUP(50),
        PUT(165),
        HOOK(-120);


        public final double deg;

        JointPositions(double deg) {
            this.deg = deg + OFFSET;
        }
    }
}
