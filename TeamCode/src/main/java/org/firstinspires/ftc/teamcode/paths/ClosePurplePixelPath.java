package org.firstinspires.ftc.teamcode.paths;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.RobotController;
import org.firstinspires.ftc.teamcode.commands.arm.MoveArmToPositionCommand;
import org.firstinspires.ftc.teamcode.commands.arm.PlacePurplePixelCommand;
import org.firstinspires.ftc.teamcode.commands.drivebase.DriveDistanceDriveCommand;
import org.firstinspires.ftc.teamcode.commands.drivebase.RotateRobotByDegCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveBaseSubsystem;
import org.firstinspires.ftc.teamcode.util.TeamColor;

public class ClosePurplePixelPath extends SequentialCommandGroup {
    public ClosePurplePixelPath(RobotController robotController, TeamColor color) {
        super();
        double rotationModifier = color == TeamColor.BLUE ? -1 : 1;
        double rotation = -45 * rotationModifier;
        double rotationKp = 0.035f;

        addCommands(
                new DriveDistanceDriveCommand(robotController.driveBase, 0.6f).withTimeout(3500),
                new RotateRobotByDegCommand(robotController.driveBase, rotation, rotationKp),
                new PlacePurplePixelCommand(robotController),
                new RotateRobotByDegCommand(robotController.driveBase, -rotation, rotationKp)
        );
    }
}
