package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Servos;

public class DroneSubsystem extends SubsystemBase {

    private final CRServo servo;
    public DroneSubsystem(HardwareMap hMap) {
        this.servo = hMap.get(CRServo.class, Servos.DRONE.id);
        this.servo.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void moveServo(double power) {
        this.servo.setPower(power);
    }
}
