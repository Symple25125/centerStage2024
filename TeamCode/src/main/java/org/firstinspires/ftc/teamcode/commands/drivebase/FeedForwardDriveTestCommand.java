package org.firstinspires.ftc.teamcode.commands.drivebase;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DriveBaseSubsystem;

@Config
public class FeedForwardDriveTestCommand extends CommandBase {
    public static double POWER = 0f;

    private final DriveBaseSubsystem driveBase;

    public FeedForwardDriveTestCommand(DriveBaseSubsystem driveBase) {
        addRequirements(driveBase);
        this.driveBase = driveBase;
    }

    @Override
    public void execute() {
        this.driveBase.moveMotors(POWER, POWER);
    }
}
