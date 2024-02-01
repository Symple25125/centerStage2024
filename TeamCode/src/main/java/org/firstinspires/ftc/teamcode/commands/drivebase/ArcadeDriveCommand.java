package org.firstinspires.ftc.teamcode.commands.drivebase;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.DriveBaseSubsystem;

public class ArcadeDriveCommand extends CommandBase {
    private final DriveBaseSubsystem driveBase;
    private final GamepadEx controller;

    public ArcadeDriveCommand(DriveBaseSubsystem driveBase, GamepadEx controller) {
        addRequirements(driveBase);
        this.driveBase = driveBase;
        this.controller = controller;
    }

    @Override
    public void execute() {
        this.driveBase.moveWithJoyStickAndNormalize(this.controller);
    }
}