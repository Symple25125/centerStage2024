package org.firstinspires.ftc.teamcode.commands.arm;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PController;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

@Config
public class MoveArmToPointWithPID extends CommandBase {
    protected final ArmSubsystem armSubsystem;
    protected final PController pController;
    private double point;
    public static double Kp = 0.005;
    protected boolean feedForward;

    public MoveArmToPointWithPID(ArmSubsystem armSubsystem, ArmSubsystem.ArmPositions pos, boolean feedForward, double kp) {
        addRequirements(armSubsystem);

        this.armSubsystem = armSubsystem;
        this.point = pos.deg;
        this.feedForward = feedForward;
        this.pController = new PController(kp);
        this.pController.setTolerance(1);
    }

    public MoveArmToPointWithPID(ArmSubsystem armSubsystem, double deg, boolean feedForward, double kp) {
        this(armSubsystem, ArmSubsystem.ArmPositions.HOOK, feedForward, kp);
        this.point = deg;
    }

    public MoveArmToPointWithPID(ArmSubsystem armSubsystem, ArmSubsystem.ArmPositions pos, boolean feedForward) {
        this(armSubsystem, pos, feedForward, Kp);
    }

    public MoveArmToPointWithPID(ArmSubsystem armSubsystem, double pos, boolean feedForward) {
        this(armSubsystem, ArmSubsystem.ArmPositions.PLACE, feedForward, Kp);
        this.point = pos;
    }

    public MoveArmToPointWithPID(ArmSubsystem armSubsystem, ArmSubsystem.ArmPositions pos) {
        this(armSubsystem, pos, true, Kp);
    }

    public MoveArmToPointWithPID(ArmSubsystem armSubsystem, double pos) {
        this(armSubsystem, pos, true, Kp);
    }

    @Override
    public void initialize() {
        this.pController.setSetPoint(this.point);
    }

    @Override
    public void execute() {
        double power = pController.calculate(this.armSubsystem.getCurrentPositionDeg());
        this.armSubsystem.moveMotor(power, feedForward);
    }

    @Override
    public void end(boolean interrupted) {
        this.armSubsystem.moveMotor(0);
    }
}
