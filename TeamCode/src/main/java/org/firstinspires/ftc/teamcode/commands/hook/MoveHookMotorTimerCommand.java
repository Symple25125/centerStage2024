package org.firstinspires.ftc.teamcode.commands.hook;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.subsystems.HookSubsystem;

import java.util.concurrent.TimeUnit;

public class MoveHookMotorTimerCommand extends CommandBase {
    private final HookSubsystem hookSubsystem;
    private final double power;
    private final Timing.Timer timer;

    public MoveHookMotorTimerCommand(HookSubsystem hookSubsystem, double power, long time) {
        addRequirements(hookSubsystem);
        this.hookSubsystem = hookSubsystem;
        this.power = power;
        this.timer = new Timing.Timer(time, TimeUnit.MILLISECONDS);
    }

    @Override
    public void initialize() {
        this.timer.start();
    }

    @Override
    public void execute() {
        super.execute();
        this.hookSubsystem.moveMotor(power);
    }

    @Override
    public void end(boolean interrupted) {
        this.hookSubsystem.moveMotor(0);
    }

    @Override
    public boolean isFinished() {
        return timer.done();
    }
}
