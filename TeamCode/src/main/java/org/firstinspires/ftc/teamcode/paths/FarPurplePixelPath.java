package org.firstinspires.ftc.teamcode.paths;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.drivebase.DriveDistanceDriveCommand;
import org.firstinspires.ftc.teamcode.commands.drivebase.RotateRobotByDegCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveBaseSubsystem;
import org.firstinspires.ftc.teamcode.util.TeamColor;

public class FarPurplePixelPath extends SequentialCommandGroup {
    public FarPurplePixelPath(DriveBaseSubsystem driveBaseSubsystem, TeamColor color) {
        super();
        double rotationModifier = color == TeamColor.BLUE ? -1 : 1;

        addCommands(
                new DriveDistanceDriveCommand(driveBaseSubsystem, 0.6f).withTimeout(3500),
                new RotateRobotByDegCommand(driveBaseSubsystem, 45 * rotationModifier, 0.035f)
        );
    }
}