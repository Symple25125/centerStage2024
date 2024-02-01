package org.firstinspires.ftc.teamcode.commands.hook;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.commands.arm.MoveArmToPointWithPID;
import org.firstinspires.ftc.teamcode.commands.joint.MoveJointToPosition;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.JointSubSystem;

public class MoveToHookPositionCommand extends ParallelCommandGroup {
    public MoveToHookPositionCommand(ArmSubsystem armSubsystem, JointSubSystem jointSubSystem) {
        addCommands(
                new MoveArmToPointWithPID(armSubsystem, ArmSubsystem.ArmPositions.HOOK),
                new MoveJointToPosition(jointSubSystem, JointSubSystem.JointPositions.HOOK)
        );
        addRequirements(armSubsystem, jointSubSystem);
    }
}
