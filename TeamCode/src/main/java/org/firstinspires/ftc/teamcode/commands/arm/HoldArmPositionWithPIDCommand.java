package org.firstinspires.ftc.teamcode.commands.arm;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PController;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

@Config
public class HoldArmPositionWithPIDCommand extends CommandBase {
    protected final ArmSubsystem armSubsystem;
    protected final PController pController;
    public static double KP = 0.01;
    public double MAX_POWER = 0.8f;

    public HoldArmPositionWithPIDCommand(ArmSubsystem armSubsystem) {
        addRequirements(armSubsystem);

        this.armSubsystem = armSubsystem;
        this.pController = new PController(KP);
        this.pController.setTolerance(1);
    }

    @Override
    public void initialize() {
        this.pController.setSetPoint(armSubsystem.getCurrentPos());
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
}
