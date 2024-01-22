package org.firstinspires.ftc.teamcode.commands.drivebase;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.arm.MoveArmToPointWithPID;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveBaseSubsystem;

import java.util.concurrent.TimeUnit;

public class MoveUnderCommand extends SequentialCommandGroup {
    private static final double movePower = -0.65;

    public MoveUnderCommand(ArmSubsystem armSubsystem, DriveBaseSubsystem driveBase) {
        addCommands(
                new MoveArmToPointWithPID(armSubsystem, ArmSubsystem.ArmPositions.UNDER_DRIVE_DOWN, 0.05),
                new MoveMotorDiveCommand(driveBase, 500, movePower, movePower, TimeUnit.MILLISECONDS),
                new MoveArmToPointWithPID(armSubsystem, ArmSubsystem.ArmPositions.UNDER_DRIVE_UP, 0.05),
                new MoveMotorDiveCommand(driveBase, 500, movePower, movePower, TimeUnit.MILLISECONDS)
        );
        addRequirements(armSubsystem, driveBase);
    }
}
