package org.firstinspires.ftc.teamcode.commands.hook;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.HookSubsystem;

@Config
public class JoyStickHookCommand extends CommandBase {
    private HookSubsystem hookSubsystem;
    private GamepadEx controller;

    public JoyStickHookCommand(HookSubsystem hookSubsystem, GamepadEx controller) {
        addRequirements(hookSubsystem);
        this.hookSubsystem = hookSubsystem;
        this.controller = controller;
    }

    @Override
    public void execute() {
        hookSubsystem.moveMotor(-controller.getLeftY());
    }
}
