package org.firstinspires.ftc.teamcode.commands.claw;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;

import java.util.concurrent.TimeUnit;

public class MoveClawToPositionSlowCommand extends CommandBase {
    private final ClawSubsystem clawSubsystem;
    private final double clawPos;
    private final double Kdeg = 15;
    private final Timing.Timer timer = new Timing.Timer(100, TimeUnit.MILLISECONDS);

    public MoveClawToPositionSlowCommand(ClawSubsystem clawSubsystem, ClawSubsystem.ClawPositions clawPosition) {
        addRequirements(clawSubsystem);
        this.clawSubsystem = clawSubsystem;
        this.clawPos = clawPosition.deg;
    }

    @Override
    public void initialize() {
        this.timer.start();
    }

    @Override
    public void execute() {
        super.execute();
        if(timer.done()) {
            double diff = this.clawPos - this.clawSubsystem.getClawDeg();
            this.clawSubsystem.moveClawToAngle(this.clawSubsystem.getClawDeg() + Math.min(this.Kdeg, Math.abs(diff)) * Math.signum(diff));
            timer.start();
        }
    }

    @Override
    public boolean isFinished() {
        return this.clawSubsystem.getClawDeg() == this.clawPos;
    }
}
