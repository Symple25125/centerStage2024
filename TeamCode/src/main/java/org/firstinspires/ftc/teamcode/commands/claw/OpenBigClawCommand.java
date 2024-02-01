package org.firstinspires.ftc.teamcode.commands.claw;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;

public class OpenBigClawCommand extends InstantCommand {
    public OpenBigClawCommand(ClawSubsystem clawSubsystem, ClawSubsystem.ClawPositions clawPositions) {
        super(() -> clawSubsystem.moveClawToPosition(clawPositions));
        addRequirements(clawSubsystem);
    }
}
