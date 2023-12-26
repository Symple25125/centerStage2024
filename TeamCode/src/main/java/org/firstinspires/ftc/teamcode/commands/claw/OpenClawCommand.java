package org.firstinspires.ftc.teamcode.commands.claw;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;

public class OpenClawCommand extends InstantCommand {
    public OpenClawCommand(ClawSubsystem clawSubsystem) {
        super(clawSubsystem::openClaw, clawSubsystem);
    }
}
