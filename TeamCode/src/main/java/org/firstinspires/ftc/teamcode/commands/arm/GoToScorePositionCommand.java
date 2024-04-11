package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.commands.claw.CloseClawCommand;
import org.firstinspires.ftc.teamcode.commands.joint.MoveJointToPosition;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.JointSubsystem;
import org.firstinspires.ftc.teamcode.util.ScorePositions;

public class GoToScorePositionCommand extends ParallelCommandGroup {
    public GoToScorePositionCommand(ClawSubsystem clawSubsystem, ArmSubsystem armSubsystem, JointSubsystem jointSubsystem, ScorePositions scorePositions) {
        super();

        addCommands(
                new CloseClawCommand(clawSubsystem),
//                new SelectCommand(
//                    new HashMap<Object, Command>() {{
//                        put(ArmSubsystem.ArmPositions.SCORE_LOWER, new MoveArmToPositionCommand(armSubsystem, ArmSubsystem.ArmPositions.SCORE_LOWER));
//                        put(ArmSubsystem.ArmPositions.SCORE_MIDDLE, new MoveArmToPositionCommand(armSubsystem, ArmSubsystem.ArmPositions.SCORE_MIDDLE));
//                        put(ArmSubsystem.ArmPositions.SCORE_UPPER, new MoveArmToPositionCommand(armSubsystem, ArmSubsystem.ArmPositions.SCORE_UPPER));
//                    }},
//                    ArmSubsystem::getNextScorePos
//                ),
                new MoveArmToPositionCommand(armSubsystem, scorePositions == ScorePositions.UPPER ? ArmSubsystem.ArmPositions.SCORE_UPPER : ArmSubsystem.ArmPositions.SCORE_LOWER),
                new MoveJointToPosition(jointSubsystem, scorePositions == ScorePositions.UPPER ? JointSubsystem.JointPositions.SCORE_UPPER : JointSubsystem.JointPositions.SCORE_LOWER)
        );
    }
}
