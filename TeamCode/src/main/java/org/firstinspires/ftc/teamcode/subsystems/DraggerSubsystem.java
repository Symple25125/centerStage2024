package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.Servos;

public class DraggerSubsystem extends SubsystemBase {
    private final ServoEx servo;

    public DraggerSubsystem(HardwareMap hMap) {
//      this.servo = new SimpleServo(hMap, "IDK", 0, 360, AngleUnit.DEGREES);
        this.servo = null;
    }


}
