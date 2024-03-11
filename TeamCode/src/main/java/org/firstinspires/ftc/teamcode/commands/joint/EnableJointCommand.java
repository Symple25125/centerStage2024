package org.firstinspires.ftc.teamcode.commands.joint;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.JointSubSystem;

public class EnableJointCommand extends InstantCommand {
    public EnableJointCommand(JointSubSystem jointSubSystem) {
        super(jointSubSystem::enableServo);
    }
}
