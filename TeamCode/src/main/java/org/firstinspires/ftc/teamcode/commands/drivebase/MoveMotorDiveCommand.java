package org.firstinspires.ftc.teamcode.commands.drivebase;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.subsystems.DriveBaseSubsystem;

import java.util.concurrent.TimeUnit;

public class MoveMotorDiveCommand extends CommandBase {
    private final DriveBaseSubsystem driveBase;

    private final double leftPower;
    private final double rightPower;
    private final Timing.Timer timer;

    public MoveMotorDiveCommand(DriveBaseSubsystem driveBase, long time, double leftPower, double rightPower, TimeUnit timeUnit) {
        addRequirements(driveBase);
        this.driveBase = driveBase;

        this.leftPower = leftPower;
        this.rightPower = rightPower;
        this.timer = new Timing.Timer(time, timeUnit);
    }

    public MoveMotorDiveCommand(DriveBaseSubsystem driveBase, long time, double leftPower, double rightPower) {
        this(driveBase, time, leftPower, rightPower, TimeUnit.SECONDS);
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {
        this.driveBase.moveMotors(this.leftPower, this.rightPower);
    }

    @Override
    public void end(boolean interrupted) {
        this.driveBase.moveMotors(0, 0);
    }

    @Override
    public boolean isFinished() {
        return timer.done();
    }
}
