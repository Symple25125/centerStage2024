package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PController;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class MoveArmToPositionCommand extends CommandBase {
    protected final ArmSubsystem armSubsystem;
    protected final PController pController;
    public static double defaultKp = 0.01;
    public double MAX_POWER = 0.8f;

    private double point;
    private final ArmSubsystem.ArmState state;

    public MoveArmToPositionCommand(ArmSubsystem armSubsystem, ArmSubsystem.ArmPositions pos, double kp) {
        addRequirements(armSubsystem);

        this.point = pos.deg;
        this.armSubsystem = armSubsystem;
        this.pController = new PController(kp);
        this.pController.setTolerance(1);

        this.state = pos.state;
    }

    public MoveArmToPositionCommand(ArmSubsystem armSubsystem, ArmSubsystem.ArmPositions pos) {
        this(armSubsystem, pos, defaultKp);
    }

    public MoveArmToPositionCommand(ArmSubsystem armSubsystem, double pos) {
        this(armSubsystem, ArmSubsystem.ArmPositions.UNKNOWN, defaultKp);

        this.point = pos;
    }

    @Override
    public void initialize() {
        ArmSubsystem.setState(this.state);
        this.pController.setSetPoint(this.point);
    }

    @Override
    public void execute() {
        double rawPower = pController.calculate(this.armSubsystem.getCurrentPos());

        double power = Math.min(Math.max(rawPower, -MAX_POWER), MAX_POWER);

        this.armSubsystem.moveMotor(power);
    }

    @Override
    public void end(boolean interrupted) {
        this.armSubsystem.moveMotor(0);
    }

    @Override
    public boolean isFinished() {
        return this.pController.atSetPoint();
    }
}
