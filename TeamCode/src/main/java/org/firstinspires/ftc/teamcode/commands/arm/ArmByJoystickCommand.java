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

        ArmSubsystem.setState(ArmSubsystem.ArmState.JOYSTICK);
    }

    @Override
    public void execute() {
        this.armSubsystem.moveMotor(-this.controller.getRightY() * 0.75f);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(this.controller.getRightY()) <= ArmSubsystem.JOYSTICK_DEAD_AREA;
    }
}
