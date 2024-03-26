package org.firstinspires.ftc.teamcode.commands.arm;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class ArmGoToRestPositionCommand extends MoveArmToPositionCommand {
    public ArmGoToRestPositionCommand(ArmSubsystem armSubsystem) {
        super(armSubsystem, ArmSubsystem.ArmPositions.REST, 0.075);
        super.MAX_POWER = 1;
    }
}
