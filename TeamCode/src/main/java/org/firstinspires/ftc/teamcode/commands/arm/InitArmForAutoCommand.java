package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

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
                new CloseClawCommand(clawSubsystem),
                new WaitCommand(500),
                new ArmGoToRestPositionCommand(armSubsystem)
                    .alongWith(new MoveJointToPosition(jointSubSystem, JointSubsystem.JointPositions.REST))
        );
    }
}
