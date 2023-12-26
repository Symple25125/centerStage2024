package org.firstinspires.ftc.teamcode.commands.drivebase;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.DriveBaseSubsystem;

public class ArcadeDriveCommand extends CommandBase {
    private final DriveBaseSubsystem driveBase;
    private final Gamepad gamepad;

    public ArcadeDriveCommand(DriveBaseSubsystem driveBase, Gamepad gamepad) {
        addRequirements(driveBase);
        this.driveBase = driveBase;
        this.gamepad = gamepad;
    }

    @Override
    public void execute() {
        float rotationSpeed = this.gamepad.right_stick_x * 0.2f;
        float linerSpeed = -this.gamepad.right_stick_y * 0.8f;

        float leftSpeed = linerSpeed - rotationSpeed;
        float rightSpeed = linerSpeed + rotationSpeed;

        this.driveBase.moveMotors(leftSpeed, rightSpeed);
    }
}