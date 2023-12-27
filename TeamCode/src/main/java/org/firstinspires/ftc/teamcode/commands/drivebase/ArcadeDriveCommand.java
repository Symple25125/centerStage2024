package org.firstinspires.ftc.teamcode.commands.drivebase;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.DriveBaseSubsystem;

public class ArcadeDriveCommand extends CommandBase {
    private final DriveBaseSubsystem driveBase;
    private final GamepadEx gamepad;

    public ArcadeDriveCommand(DriveBaseSubsystem driveBase, GamepadEx gamepad) {
        addRequirements(driveBase);
        this.driveBase = driveBase;
        this.gamepad = gamepad;
    }

    @Override
    public void execute() {
        double rotationSpeed = this.gamepad.getRightX() * 0.2f;
        double linerSpeed = -this.gamepad.getRightY() * 0.8f;

        double leftSpeed = linerSpeed - rotationSpeed;
        double rightSpeed = linerSpeed + rotationSpeed;

        this.driveBase.moveMotors(leftSpeed, rightSpeed);
    }
}