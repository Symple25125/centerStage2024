package org.firstinspires.ftc.teamcode.commands.joint;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.JointSubSystem;

public class DisableJointCommand extends InstantCommand {
    public DisableJointCommand(JointSubSystem jointSubSystem) {
        super(jointSubSystem::disableServo);
    }
}
