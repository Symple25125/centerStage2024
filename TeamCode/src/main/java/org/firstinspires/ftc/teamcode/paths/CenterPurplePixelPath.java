package org.firstinspires.ftc.teamcode.paths;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.RobotController;
import org.firstinspires.ftc.teamcode.commands.arm.PlacePurplePixelCommand;
import org.firstinspires.ftc.teamcode.commands.drivebase.DriveDistanceDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveBaseSubsystem;

public class CenterPurplePixelPath extends SequentialCommandGroup {
    public CenterPurplePixelPath(RobotController robotController) {
        super();
        double fromCenter = 0.15f;

        addCommands(
                new DriveDistanceDriveCommand(robotController.driveBase, 0.6f + fromCenter).withTimeout(4500),
                new PlacePurplePixelCommand(robotController),
                new DriveDistanceDriveCommand(robotController.driveBase, -fromCenter).withTimeout(1500)
        );
    }
}