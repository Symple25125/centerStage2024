package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.Servos;
import org.firstinspires.ftc.teamcode.util.SympleServo;

@Config
public class JointSubsystem extends SubsystemBase {
    private final SympleServo servo;
    public static final double OFFSET = 0;

    public JointSubsystem(HardwareMap hMap) {
        this.servo = new SympleServo(hMap, Servos.CLAW_JOINT.id, OFFSET - 135, OFFSET + 135, AngleUnit.DEGREES);
    }

    public void disableServo() {
        this.servo.disable();
    }
    public void enableServo() {
        this.servo.enable();
    }

    public void moveServo(JointPositions position) {
        servo.turnToAngle(position.deg);
    }

    public double getClawJointAngle() {
        return servo.getAngle(AngleUnit.DEGREES);
    }

    public enum JointPositions {
        PICKUP(-35),
        SCORE_UPPER(95),
        SCORE_LOWER(75),
        HOOK(-180),
        ZERO(0),
        REST(-10);


        public final double deg;

        JointPositions(double deg) {
            this.deg = deg + OFFSET;
        }
    }
}
