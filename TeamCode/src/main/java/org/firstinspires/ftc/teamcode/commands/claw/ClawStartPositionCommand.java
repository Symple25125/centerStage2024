package org.firstinspires.ftc.teamcode.commands.claw;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;

public class ClawStartPositionCommand extends InstantCommand {
    public ClawStartPositionCommand(ClawSubsystem clawSubsystem) {
        super(() -> clawSubsystem.moveClawToPosition(ClawSubsystem.ClawPositions.OPEN) , clawSubsystem);
    }
}
