package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.claw.CloseClawCommand;
import org.firstinspires.ftc.teamcode.commands.joint.EnableJointCommand;
import org.firstinspires.ftc.teamcode.commands.joint.MoveJointToPosition;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.JointSubsystem;

public class InitArmForAutoCommand extends SequentialCommandGroup {
    public InitArmForAutoCommand(JointSubsystem jointSubSystem, ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem) {
        super();

        addCommands(
                new EnableJointCommand(jointSubSystem),
                new MoveArmToPositionCommand(armSubsystem, ArmSubsystem.ArmPositions.PICKUP)
                        .alongWith(new MoveJointToPosition(jointSubSystem, JointSubsystem.JointPositions.PICKUP))
                        .withTimeout(500)
                        .andThen(new CloseClawCommand(clawSubsystem))
        );
    }
}
