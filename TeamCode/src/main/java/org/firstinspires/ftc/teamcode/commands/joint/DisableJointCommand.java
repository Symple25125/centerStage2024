package org.firstinspires.ftc.teamcode.commands.joint;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.JointSubsystem;

public class DisableJointCommand extends InstantCommand {
    public DisableJointCommand(JointSubsystem jointSubSystem) {
        super(jointSubSystem::disableServo);
    }
}
