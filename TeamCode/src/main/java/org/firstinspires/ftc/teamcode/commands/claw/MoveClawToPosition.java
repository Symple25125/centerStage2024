package org.firstinspires.ftc.teamcode.commands.claw;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;

public class MoveClawToPosition extends InstantCommand {
    public MoveClawToPosition(ClawSubsystem clawSubsystem, ClawSubsystem.ClawPositions pos) {
        super(() -> clawSubsystem.moveClawToPosition(pos), clawSubsystem);
    }
}
