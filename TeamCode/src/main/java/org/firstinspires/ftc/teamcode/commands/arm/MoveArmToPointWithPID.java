package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PController;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class MoveArmToPointWithPID extends CommandBase {
    private final ArmSubsystem armSubsystem;
    private final double point;

    public MoveArmToPointWithPID(ArmSubsystem armSubsystem, double deg) {
        addRequirements(armSubsystem);

        this.armSubsystem = armSubsystem;
        this.point = deg;
    }

    @Override
    public void execute() {
        this.armSubsystem.moveToPoint(this.point);
    }

    @Override
    public boolean isFinished() {
        return this.armSubsystem.isAtPosition();
    }
}
