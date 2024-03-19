package org.firstinspires.ftc.teamcode.paths;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.DriveBaseSubsystem;
import org.firstinspires.ftc.teamcode.util.DetectionSide;
import org.firstinspires.ftc.teamcode.util.TeamColor;

import java.util.HashMap;

public class Paths {

    public static SequentialCommandGroup generatePath(TeamColor teamColor, DetectionSide detectionSide, DriveBaseSubsystem driveBaseSubsystem) {
         switch(detectionSide) {
            case FAR: {
                return new FarPurplePixelPath(driveBaseSubsystem, teamColor);
            }

            case CENTER: {
                return new CenterPurplePixelPath(driveBaseSubsystem);
            }

            case CLOSE: {
                return new ClosePurplePixelPath(driveBaseSubsystem, teamColor);
            }
        }

        return new SequentialCommandGroup();
    }
}
