package org.firstinspires.ftc.teamcode.commands.hook;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.commands.joint.MoveJointToPosition;
import org.firstinspires.ftc.teamcode.subsystems.HookSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.JointSubSystem;

public class AutoHookCloseCommand extends ParallelCommandGroup {
    public AutoHookCloseCommand(HookSubsystem hookSubsystem, JointSubSystem jointSubSystem) {
        addCommands(
                new MoveJointToPosition(jointSubSystem, JointSubSystem.JointPositions.PUT),
                new MoveHookMotorCommand(hookSubsystem, 1)
        );
        addRequirements(hookSubsystem, jointSubSystem);
    }
}
