package org.firstinspires.ftc.teamcode.commands.claw;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;

public class FullyOpenClawCommand extends InstantCommand {
    public FullyOpenClawCommand(ClawSubsystem clawSubsystem) {
        super(() -> clawSubsystem.moveClawToPosition(ClawSubsystem.ClawPositions.FULL_OPEN));
        addRequirements(clawSubsystem);
    }
}
