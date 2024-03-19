package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.claw.CloseClawCommand;
import org.firstinspires.ftc.teamcode.commands.joint.EnableJointCommand;
import org.firstinspires.ftc.teamcode.commands.joint.MoveJointToPosition;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.JointSubSystem;

public class InitArmForAutoCommand extends SequentialCommandGroup {
    public InitArmForAutoCommand(JointSubSystem jointSubSystem, ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem) {
        super();

        addRequirements(jointSubSystem, armSubsystem, clawSubsystem);

        addCommands(
                new EnableJointCommand(jointSubSystem),
                new GoToPickupPositionCommand(armSubsystem)
                        .alongWith(new MoveJointToPosition(jointSubSystem, JointSubSystem.JointPositions.PICKUP))
                        .withTimeout(500)
                        .andThen(new CloseClawCommand(clawSubsystem))
        );
    }
}
