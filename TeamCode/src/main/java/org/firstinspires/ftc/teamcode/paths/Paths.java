package org.firstinspires.ftc.teamcode.paths;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.RobotController;
import org.firstinspires.ftc.teamcode.subsystems.DriveBaseSubsystem;
import org.firstinspires.ftc.teamcode.util.DetectionSide;
import org.firstinspires.ftc.teamcode.util.TeamColor;

public class Paths {

    public static SequentialCommandGroup generatePath(RobotController robotController, DetectionSide detectionSide) {
        if(detectionSide == null) detectionSide = DetectionSide.CLOSE;

        switch(detectionSide) {
            case FAR: {
                return new FarPurplePixelPath(robotController, robotController.getTeamColor());
            }

            case CENTER: {
                return new CenterPurplePixelPath(robotController);
            }

            case CLOSE: {
                return new ClosePurplePixelPath(robotController, robotController.getTeamColor());
            }
        }

        return new SequentialCommandGroup();
    }
}
