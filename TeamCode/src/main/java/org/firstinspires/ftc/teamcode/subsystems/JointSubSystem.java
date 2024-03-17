package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.Servos;
import org.firstinspires.ftc.teamcode.util.SympleServo;

@Config
public class JointSubSystem extends SubsystemBase {
    private final SympleServo servo;
    public static final double OFFSET = 0;

    public JointSubSystem(HardwareMap hMap) {
        this.servo = new SympleServo(hMap, Servos.CLAW_JOINT.id, OFFSET - 180, OFFSET + 180, AngleUnit.DEGREES);
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
        PICKUP(50),
        PUT(195),
        HOOK(-120),
        ZERO(0),
        REST(95);


        public final double deg;

        JointPositions(double deg) {
            this.deg = deg + OFFSET;
        }
    }
}
