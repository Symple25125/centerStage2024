package org.firstinspires.ftc.teamcode.commands.arm;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class RobotGoUpCommand extends MoveArmToPositionCommand {
    public RobotGoUpCommand(ArmSubsystem armSubsystem){
        super(armSubsystem, ArmSubsystem.ArmPositions.GRAB, 0.5f);
        super.MAX_POWER = 1;
    }

    @Override
    public void execute() {
        double rawPower = pController.calculate(this.armSubsystem.getCurrentPos());

        double power = Math.min(Math.max(rawPower, -MAX_POWER), MAX_POWER);

        this.armSubsystem.moveMotor(power);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

