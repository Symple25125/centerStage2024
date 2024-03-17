package org.firstinspires.ftc.teamcode.commands.arm;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class GoToPickupPositionCommand extends MoveArmToPointWithPID {
//    private static final double EXTRA_POWER = -0.085;
    private static final double EXTRA_POWER = 0;

    public GoToPickupPositionCommand(ArmSubsystem armSubsystem) {
        super(armSubsystem, ArmSubsystem.ArmPositions.TAKE);
    }

    @Override
    public void execute() {
        double power = pController.calculate(this.armSubsystem.getCurrentPos());
        this.armSubsystem.moveMotor(power + (this.armSubsystem.getCurrentPos() <= -20 ? EXTRA_POWER : 0));
    }
}
