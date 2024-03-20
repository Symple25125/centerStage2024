package org.firstinspires.ftc.teamcode.commands.claw;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;

public class ToggleClawCommand extends InstantCommand {
    public ToggleClawCommand(ClawSubsystem clawSubsystem) {
        super(() -> {
            if (ClawSubsystem.getCurrentState() == ClawSubsystem.ClawState.OPEN) {
                clawSubsystem.closeClaw();
            } else {
                clawSubsystem.openClaw();
            }
        });
    }
}
