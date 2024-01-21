package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class ArmByJoystickCommand extends CommandBase {
    private final GamepadEx controller;
    private final ArmSubsystem armSubsystem;

    public ArmByJoystickCommand(ArmSubsystem armSubsystem, GamepadEx controller) {
        addRequirements(armSubsystem);
        this.armSubsystem = armSubsystem;
        this.controller = controller;
    }

    @Override
    public void execute() {
        this.armSubsystem.moveMotor(-this.controller.getRightY() * 0.5);
    }
}
