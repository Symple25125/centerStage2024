package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.commands.claw.CloseClawCommand;
import org.firstinspires.ftc.teamcode.commands.joint.MoveJointToPosition;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.JointSubsystem;

public class GoToScorePositionCommand extends ParallelCommandGroup {
    public GoToScorePositionCommand(ClawSubsystem clawSubsystem, ArmSubsystem armSubsystem, JointSubsystem jointSubsystem, ArmSubsystem.ArmPositions pos) {
        super();

        addCommands(
                new CloseClawCommand(clawSubsystem),
                new MoveArmToPositionCommand(armSubsystem, pos),
                new MoveJointToPosition(jointSubsystem, JointSubsystem.JointPositions.PUT)
        );
    }
}
