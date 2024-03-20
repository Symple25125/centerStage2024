package org.firstinspires.ftc.teamcode.commands.joint;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.JointSubsystem;

public class EnableJointCommand extends InstantCommand {
    public EnableJointCommand(JointSubsystem jointSubSystem) {
        super(jointSubSystem::enableServo);
    }
}
