package org.firstinspires.ftc.teamcode.commands.drivebase;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

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
        double speedModifier = 1 - (this.controller.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) * DriveBaseSubsystem.MIN_CONTROLLER_SPEED_MODIFIER);

        double rotationSpeed = controller.getRightX() * 0.5f * (this.driveBase.isInverted() ? -1 : 1) * speedModifier;
        double linerSpeed = controller.getLeftY() * 0.8f * (this.driveBase.isInverted() ? -1 : 1) * speedModifier;

        double rawLeftSpeed = (linerSpeed + rotationSpeed);
        double rawRightSpeed = (linerSpeed - rotationSpeed);

        double normalizedLeftSpeed = rawLeftSpeed;
        double normalizedRightSpeed = rawRightSpeed;


        double absRightSpeed = Math.abs(rawRightSpeed);
        double absLeftSpeed = Math.abs(rawLeftSpeed);
        double maxValue = Math.max(absLeftSpeed, absRightSpeed);

        if(maxValue >= 1) {
            normalizedLeftSpeed = rawLeftSpeed / maxValue;
            normalizedRightSpeed = rawRightSpeed / maxValue;
        }

        this.driveBase.moveMotors(normalizedLeftSpeed, normalizedRightSpeed);
    }
}