package org.firstinspires.ftc.teamcode.commands.hook;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.HookSubsystem;

public class MoveHookMotorCommand extends CommandBase {
    private final HookSubsystem hookSubsystem;
    private final double power;

    public MoveHookMotorCommand(HookSubsystem hookSubsystem, double power) {
        addRequirements(hookSubsystem);
        this.hookSubsystem = hookSubsystem;
        this.power = power;
    }

    @Override
    public void execute() {
        this.hookSubsystem.moveMotor(power);
    }
}
