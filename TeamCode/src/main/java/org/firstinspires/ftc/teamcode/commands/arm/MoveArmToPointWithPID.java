package org.firstinspires.ftc.teamcode.commands.arm;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PController;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class MoveArmToPointWithPID extends CommandBase {
    private final ArmSubsystem armSubsystem;
    private final PController pController;
    private final double point;
    private int timeFinished = 0;
    private int maxFinish = 3;

    private static double Kp = 0.0075;

    public MoveArmToPointWithPID(ArmSubsystem armSubsystem, ArmSubsystem.ArmPositions pos, double kp) {
        addRequirements(armSubsystem);

        this.armSubsystem = armSubsystem;
        this.point = pos.deg;
        this.pController = new PController(kp);
        this.pController.setTolerance(1);
    }

    public MoveArmToPointWithPID(ArmSubsystem armSubsystem, ArmSubsystem.ArmPositions pos) {
        this(armSubsystem, pos, Kp);
    }

    @Override
    public void initialize() {
        this.pController.setSetPoint(this.point);
    }

    @Override
    public void execute() {
        double power = pController.calculate(this.armSubsystem.getCurrentPositionDeg());
        this.armSubsystem.moveMotor(power);
    }

    @Override
    public void end(boolean interrupted) {
        this.armSubsystem.moveMotor(0);
    }

    @Override
    public boolean isFinished() {
        if(this.pController.atSetPoint()) this.timeFinished += 1;
            else this.timeFinished = 0;

        return this.timeFinished >= maxFinish;
    }
}
