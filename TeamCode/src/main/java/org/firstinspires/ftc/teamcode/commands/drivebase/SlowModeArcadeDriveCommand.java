package org.firstinspires.ftc.teamcode.commands.drivebase;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.DriveBaseSubsystem;
import org.opencv.core.Mat;

public class SlowModeArcadeDriveCommand extends CommandBase {
    private final DriveBaseSubsystem driveBase;
    private final GamepadEx controller;

    private final double SPEED_MODIFIER = 0.5;

    public SlowModeArcadeDriveCommand(DriveBaseSubsystem driveBase, GamepadEx gamepad) {
        addRequirements(driveBase);
        this.driveBase = driveBase;
        this.controller = gamepad;
    }

    @Override
    public void execute() {
        this.driveBase.moveWithJoyStickAndNormalize(this.controller, SPEED_MODIFIER, 1);
    }
}
