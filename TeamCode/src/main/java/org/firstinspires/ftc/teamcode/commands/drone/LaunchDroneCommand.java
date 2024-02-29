package org.firstinspires.ftc.teamcode.commands.drone;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.subsystems.DroneSubsystem;

import java.util.concurrent.TimeUnit;

public class LaunchDroneCommand extends CommandBase {
    private final DroneSubsystem droneSubsystem;
    private final Timing.Timer timer;
    public LaunchDroneCommand(DroneSubsystem droneSubsystem) {
        addRequirements(droneSubsystem);
        this.droneSubsystem = droneSubsystem;
        this.timer = new Timing.Timer(2000, TimeUnit.MILLISECONDS);
    }

    @Override
    public void initialize() {
        this.timer.start();
    }

    @Override
    public void execute() {
        this.droneSubsystem.moveServo(1);
    }

    @Override
    public void end(boolean interrupted) {
        this.droneSubsystem.moveServo(0);
    }

    @Override
    public boolean isFinished() {
        return this.timer.done();
    }
}
