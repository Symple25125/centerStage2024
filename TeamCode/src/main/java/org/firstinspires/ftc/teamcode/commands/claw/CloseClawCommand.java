package org.firstinspires.ftc.teamcode.commands.claw;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;

public class CloseClawCommand extends InstantCommand {
    public CloseClawCommand(ClawSubsystem clawSubsystem) {
        super(clawSubsystem::closeClaw , clawSubsystem);
    }
}
