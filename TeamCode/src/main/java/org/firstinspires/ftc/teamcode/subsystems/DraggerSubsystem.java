package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DraggerSubsystem extends SubsystemBase {
    private final ServoEx servo;

    public DraggerSubsystem(HardwareMap hMap) {
//      this.servo = new SimpleServo(hMap, "IDK", 0, 360, AngleUnit.DEGREES);
        this.servo = null;
    }


}
